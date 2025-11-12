import numpy as np
import cv2
import matplotlib.pyplot as plt

def load_radar(raw_img):
    raw_data = np.asarray(raw_img)
    time_convert = 1 # Keep timestamps as nanoseconds
    encoder_conversion = 2 * np.pi / 5600
    timestamps = np.frombuffer(raw_data[:,:8].tobytes(), dtype=np.int64) * time_convert
    azimuths = np.frombuffer(raw_data[:,8:10].tobytes(), dtype=np.uint16) * encoder_conversion
    up_chrips = raw_data[:,10]
    fft_data = np.divide(raw_data[:,11:], 255.0, dtype=np.float32)
    return fft_data, azimuths, timestamps, up_chrips


def cfar_mask(raw_scan, radar_res, width=137, minr=2.0, maxr=80.0,
              guard=7, a_thresh=1.0, b_thresh=0.13):
    """
    Apply 1D GO-CFAR to each azimuth of a radar scan.
    """
    assert raw_scan.ndim == 2, "raw_scan must be 2D (azimuth x range)"

    # Handle odd width
    width = width + 1 if width % 2 == 0 else width
    w2 = width // 2

    # Compute column range based on minimum/maximum range
    mincol = max(0, int(minr / radar_res + w2 + guard + 1))
    maxcol = min(raw_scan.shape[1], int(maxr / radar_res - w2 - guard))
    col_range = np.arange(mincol, maxcol)

    # Preallocate arrays
    left = np.zeros((raw_scan.shape[0], len(col_range)))
    right = np.zeros((raw_scan.shape[0], len(col_range)))

    # Compute GO-CFAR statistics
    for idx, c in enumerate(col_range):
        left_start = c - w2 - guard
        left_end = c - guard
        right_start = c + guard + 1
        right_end = c + w2 + guard + 1

        left[:, idx] = np.sum(raw_scan[:, left_start:left_end], axis=1)
        right[:, idx] = np.sum(raw_scan[:, right_start:right_end], axis=1)

    stat = np.maximum(left, right) / w2  # GO-CFAR
    thres = a_thresh * stat + b_thresh

    # Save full-sized threshold map
    thres_full = 1000 * np.ones_like(raw_scan)
    thres_full[:, col_range] = thres

    # Compute threshold mask
    thres_mask = (raw_scan > thres_full).astype(float)

    return thres_mask


def extract_pc(thres_mask, radar_res, azimuth_angles, azimuth_times,
               vel_input=None, T_ab=None):
    """
    Convert CFAR mask into a list of point clouds, one per azimuth.
    """
    assert thres_mask.ndim == 2, "thres_mask must be 2D (azimuth x range)"

    # Thresholded scan
    thres_scan = radar_res * np.arange(thres_mask.shape[1]) * thres_mask

    # Extract peak pairs
    peak_points = mean_peaks_parallel_fast(thres_scan)

    # Build per-azimuth matrices
    azimuth_angles_mat = np.tile(azimuth_angles[:, np.newaxis], (1, thres_mask.shape[1]))
    azimuth_times_mat = np.tile(azimuth_times[:, np.newaxis], (1, thres_mask.shape[1]))
    azimuth_index_mat = np.tile(np.arange(thres_mask.shape[0])[:, np.newaxis], (1, thres_mask.shape[1]))

    # Combine features
    if vel_input is not None:
        if vel_input.ndim == 1:
            azimuth_vel_mat = np.tile(vel_input[:, np.newaxis], (1, thres_mask.shape[1]))
        else:
            azimuth_vel_mat = vel_input
        peak_pt_mat = np.stack(
            (peak_points, azimuth_angles_mat, azimuth_times_mat, azimuth_vel_mat, azimuth_index_mat),
            axis=-1
        )
    else:
        peak_pt_mat = np.stack(
            (peak_points, azimuth_angles_mat, azimuth_times_mat, azimuth_index_mat),
            axis=-1
        )

    # Flatten per azimuth
    pc_list = []
    for ii in range(peak_pt_mat.shape[0]):
        peak_pt_vec_ii = peak_pt_mat[ii]
        nonzero_indices = np.nonzero(peak_pt_vec_ii[:, 0])[0]

        # Take midpoints between start and end peaks
        nonzero_even = nonzero_indices[0::2]
        nonzero_odd = nonzero_indices[1::2]
        if len(nonzero_even) == 0 or len(nonzero_odd) == 0:
            continue

        start_pts = peak_pt_vec_ii[nonzero_even]
        end_pts = peak_pt_vec_ii[nonzero_odd]
        avg_pts = (start_pts + end_pts) / 2.0

        # Convert to Cartesian
        pc_ii = pol_2_cart(avg_pts)
        if T_ab is not None:
            T_ii = T_ab[ii]
            pc_ii = (T_ii[:3, :3] @ pc_ii.T).T + T_ii[:3, 3]
        pc_list.append(pc_ii)

    return np.vstack(pc_list)


def mean_peaks_parallel_fast(arr, return_all=False):
    """Find start/end peaks in a 2D thresholded radar scan."""
    assert arr.ndim == 2, "arr must be 2D (azimuth x range)"

    res = np.zeros_like(arr)
    zero_detect_arr = arr == 0

    # First and last nonzero values per blob
    res_forward = arr[:, :-1] * (zero_detect_arr[:, 1:])
    res_backward = arr[:, 1:] * (zero_detect_arr[:, :-1])

    # Combine both (shifted into matching columns)
    res[:, :-1] = res_forward + res_backward

    if return_all:
        return res, res_forward, res_backward
    return res


def pol_2_cart(pointcloud):
    """Convert polar (rho, phi, ...) to Cartesian (x, y, z, ...)."""
    rho = pointcloud[:, 0]
    phi = pointcloud[:, 1]

    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    z = np.zeros_like(rho)

    # Handle optional velocity/index columns
    if pointcloud.shape[1] == 3:
        return np.stack((x, y, z), axis=1)
    else:
        return np.stack((x, y, z, pointcloud[:, 3], pointcloud[:, 4]), axis=1)
    

def radar_polar_to_cartesian(fft_data, azimuths, radar_resolution, cart_resolution=0.2384, cart_pixel_width=640,
                             interpolate_crossover=False, fix_wobble=True):
    # TAKEN FROM PYBOREAS
    """Convert a polar radar scan to cartesian.
    Args:
        azimuths (np.ndarray): Rotation for each polar radar azimuth (radians)
        fft_data (np.ndarray): Polar radar power readings
        radar_resolution (float): Resolution of the polar radar data (metres per pixel)
        cart_resolution (float): Cartesian resolution (metres per pixel)
        cart_pixel_width (int): Width and height of the returned square cartesian output (pixels)
        interpolate_crossover (bool, optional): If true interpolates between the end and start  azimuth of the scan. In
            practice a scan before / after should be used but this prevents nan regions in the return cartesian form.

    Returns:
        np.ndarray: Cartesian radar power readings
    """
    # Compute the range (m) captured by pixels in cartesian scan
    if (cart_pixel_width % 2) == 0:
        cart_min_range = (cart_pixel_width / 2 - 0.5) * cart_resolution
    else:
        cart_min_range = cart_pixel_width // 2 * cart_resolution
    # Compute the value of each cartesian pixel, centered at 0
    coords = np.linspace(-cart_min_range, cart_min_range, cart_pixel_width, dtype=np.float32)

    Y, X = np.meshgrid(coords, -1 * coords)
    sample_range = np.sqrt(Y * Y + X * X)
    sample_angle = np.arctan2(Y, X)
    sample_angle += (sample_angle < 0).astype(np.float32) * 2. * np.pi

    # Interpolate Radar Data Coordinates
    azimuth_step = (azimuths[-1] - azimuths[0]) / (azimuths.shape[0] - 1)
    sample_u = (sample_range - radar_resolution / 2) / radar_resolution
    sample_v = (sample_angle - azimuths[0]) / azimuth_step
    # This fixes the wobble in the old CIR204 data from Boreas
    M = azimuths.shape[0]
    azms = azimuths.squeeze()
    if fix_wobble:
        c3 = np.searchsorted(azms, sample_angle.squeeze())
        c3[c3 == M] -= 1
        c2 = c3 - 1
        c2[c2 < 0] += 1
        a3 = azms[c3]
        diff = sample_angle.squeeze() - a3
        a2 = azms[c2]
        delta = diff * (diff < 0) * (c3 > 0) / (a3 - a2 + 1e-14)
        sample_v = (c3 + delta).astype(np.float32)

    # We clip the sample points to the minimum sensor reading range so that we
    # do not have undefined results in the centre of the image. In practice
    # this region is simply undefined.
    sample_u[sample_u < 0] = 0

    if interpolate_crossover:
        fft_data = np.concatenate((fft_data[-1:], fft_data, fft_data[:1]), 0)
        sample_v = sample_v + 1

    polar_to_cart_warp = np.stack((sample_u, sample_v), -1)
    return cv2.remap(fft_data, polar_to_cart_warp, None, cv2.INTER_LINEAR)


def point_to_cart_idx(pc, cart_resolution=0.2384, cart_pixel_width=640, min_to_plus_1=False):
    # Compute the cartesian pixel coordinates of each point in the pointcloud pc
    # pc is a tensor of shape (N, m, 2/3)

    # First, isolate the x and y coordinates of the scan_pc points
    # and convert to pixels. Note, we want the x axis to point up
    # and the y axis to point right. Since indexing is from top to bottom,
    # flip the x/u indexing
    grid_pc_u = -pc[:,:,0] / cart_resolution
    grid_pc_v = pc[:,:,1] / cart_resolution

    if min_to_plus_1:
        # grid_sample requires the grid to list x/y coordinates in the range [-1, 1]
        # not 100% sure why, but need to stack the x/v and y/u coordinates in the opposite order
        grid_pc = np.stack((grid_pc_v, grid_pc_u), axis=2)
        # Normalize grid_pc to be in the range [-1, 1] (this is needed for grid_sample)
        # grid_pc is already centered at 0, so we simply need to normalize
        grid_pc = grid_pc / (cart_pixel_width - 1) * 2
    else:
        grid_pc = np.stack((grid_pc_u, grid_pc_v), axis=2)
        # Align (0, 0) with the center of the pixel grid
        grid_pc = grid_pc + cart_pixel_width / 2

    return grid_pc


def visualize_doppler(cart_img, pc, vmax=20, vmin=-20, start_fig=True, show_colourbar=True, cart_resolution=0.2384):
    # Need to align the cartesian image with pointcloud xy coordinates
    cart_img = np.rot90(cart_img, k=1, axes=(0,1))
    cart_img = np.flip(cart_img, axis=0)

    # Get cartesian indices of pointcloud
    pc_cart_idx = point_to_cart_idx(np.expand_dims(pc, 0), cart_resolution=cart_resolution, cart_pixel_width=cart_img.shape[0]).squeeze(0)
    non_nan = ~np.isnan(pc[:, 3])

    # Plot cartesian with overlaid points
    if start_fig: fig = plt.figure()
    img = plt.imshow(cart_img, cmap='gray')
    plt.scatter(pc_cart_idx[non_nan,0], pc_cart_idx[non_nan,1], s=5, c=pc[non_nan,3], cmap='bwr_r', vmax=vmax, vmin=vmin, rasterized=True)
    plt.scatter(pc_cart_idx[~non_nan,0], pc_cart_idx[~non_nan,1], s=5, c='orange', marker='D', rasterized=True)

    # Plot arrows from each point to center of image of length equal to velocity
    DX = (cart_img.shape[0]/2 - pc_cart_idx[non_nan,0])
    DY = (cart_img.shape[1]/2 - pc_cart_idx[non_nan,1])
    arrow_vectors = np.stack((DX, DY), axis=1)
    arrow_vectors = 0.005*np.expand_dims(pc[non_nan,3], 1) * arrow_vectors / np.linalg.norm(arrow_vectors, axis=1, keepdims=True)
    plt.quiver(pc_cart_idx[non_nan,0], pc_cart_idx[non_nan,1], arrow_vectors[:,0], arrow_vectors[:,1], pc[non_nan,3], angles='xy', cmap='bwr_r', clim=(vmin, vmax), scale=1, edgecolor='white', linewidth = 0.0, width=0.005, headwidth=3, headlength=4, headaxislength=3, rasterized=True)

    if show_colourbar:
        plt.colorbar(fraction=0.03, pad=0.04, label='Velocity Magnitude (m/s)')

    plt.xlim([0, cart_img.shape[0]])
    plt.ylim([0, cart_img.shape[1]])
    
    # Remove x and y ticks
    plt.xticks([])
    plt.yticks([])
    
    return img