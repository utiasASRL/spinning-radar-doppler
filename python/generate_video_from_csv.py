#!/usr/bin/env python3
"""
generate_doppler_velocity_video.py

Script to create a video overlaying Doppler velocities on radar scans.
Takes as input:
    --radar_data_path : Path to the radar scan directory
    --csv_path        : Path to the Doppler velocity CSV file
    --output_path     : Directory where the output video will be saved
    --config_path     : Path to configuration file
"""


import os
import pandas as pd
import os.path as osp
import cv2
from utils import load_radar, cfar_mask, extract_pc, radar_polar_to_cartesian, visualize_doppler
import yaml
import numpy as np
import matplotlib.pyplot as plt
from tqdm import tqdm
from matplotlib import rcParams
import argparse
from concurrent.futures import ProcessPoolExecutor
import matplotlib
matplotlib.use("Agg")  # safe, non-interactive backend for parallel plots

rcParams['pdf.fonttype'] = 42
rcParams['ps.fonttype'] = 42

def parse_args():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Create a video overlaying Doppler velocities on radar scans."
    )
    parser.add_argument("--radar_data_path", required=True, type=str,
                        help="Path to directory containing radar scans (.png).")
    parser.add_argument("--csv_path", required=True, type=str,
                        help="Path to Doppler velocity CSV file.")
    parser.add_argument("--output_path", required=True, type=str,
                        help="Directory to save the output video.")
    parser.add_argument("--config_path", required=True, type=str,
                        help="Path to configuration YAML file.")
    return parser.parse_args()


def generate_video(root_dir, output_dir, video_name, rotate=False, H=400, W=400):
    files = os.listdir(root_dir, )
    img_files = [f for f in files if 'png' in f]
    img_files.sort()

    frame_rate = 30

    out = cv2.VideoWriter(output_dir + '/' + video_name + '.mp4', cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), frame_rate, (W, H))

    for i in tqdm(range(len(img_files)), desc="Extracting"):
        frame = cv2.imread(root_dir + '/' + img_files[i])
        # Don't crop - just resize directly to preserve the full image
        frame = cv2.resize(frame, (W, H))
        if rotate:
            frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        out.write(frame)

    out.release()
    cv2.destroyAllWindows()


def _process_one_frame(task):
    (img_path, timestamp, radar_res, max_range, cart_resolution,
     df_by_frame, cart_dir, dopp_est_dir) = task

    img_name = osp.basename(img_path)[:-4]

    # Load raw radar data
    radar_img = cv2.imread(img_path, cv2.IMREAD_GRAYSCALE)
    fft_data, azimuths, timestamps, up_chirps = load_radar(radar_img)

    # Velocity data for this frame
    frame_rows = df_by_frame.get(timestamp, None)
    if frame_rows is None:
        # No rows for this timestamp; skip gracefully
        return False

    dopp_timestamps = frame_rows["dopp_timestamp"]
    dopp_velocity  = frame_rows["dopp_velocity"]
    azimuth_idx    = frame_rows["azimuth_idx"]

    # Build per-azimuth radial velocity (NaN where missing)
    u_est = np.nan * np.ones_like(azimuths)
    for i, idx in enumerate(azimuth_idx):
        u_est[idx] = dopp_velocity[i]

    # CFAR + PC extraction
    cfar_fft = cfar_mask(fft_data, radar_res, a_thresh=0.7, b_thresh=0.21, maxr=max_range)
    pc = extract_pc(cfar_fft, radar_res, azimuths, timestamps, u_est)  # returns stacked (N, 5)

    # Convert to cartesian
    cart_pixel_width = int(2 * max_range / cart_resolution)
    radar_cart_img = radar_polar_to_cartesian(
        fft_data, azimuths, radar_res,
        cart_resolution=cart_resolution, cart_pixel_width=cart_pixel_width
    )

    # Save grayscale cartesian image
    plt.figure()
    plt.imshow(radar_cart_img, cmap='gray')
    plt.gca().set_axis_off()
    plt.savefig(osp.join(cart_dir, img_name + ".png"), bbox_inches='tight', pad_inches=0)
    plt.close()

    # Save Doppler overlay
    vmax, vmin = 20, -20
    plt.figure()
    plt.gca().set_axis_off()
    visualize_doppler(
        radar_cart_img, pc, start_fig=False, show_colourbar=False,
        vmax=vmax, vmin=vmin, cart_resolution=cart_resolution
    )
    plt.savefig(osp.join(dopp_est_dir, img_name + ".png"), bbox_inches='tight', pad_inches=0, dpi=250)
    plt.close()

    return True


def main():
    args = parse_args()

    radar_dir = osp.abspath(args.radar_data_path)
    csv_path = osp.abspath(args.csv_path)
    output_dir = osp.abspath(args.output_path)
    config_path = osp.abspath(args.config_path)
    video_path = osp.join(output_dir, "doppler_overlay.mp4")

    print(f"Radar data path: {radar_dir}")
    print(f"CSV path:        {csv_path}")
    print(f"Output path:     {video_path}")
    print(f"Config path:     {config_path}")

    # Load in config
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    radar_res = config['radar']['radar_res']
    max_range = config['extraction']['signal']['max_range']
    cart_resolution = 0.2384  # metres per pixel, can be moved to config I guess

    # Load in data
    img_list = sorted(os.listdir(radar_dir))
    img_list = [f for f in img_list if '.png' in f]
    len_radar = len(img_list)
    df = pd.read_csv(csv_path)
    len_dopp_scans = df["frame_timestamp"].nunique()
    
    print(f"Found {len_radar} radar images")
    print(f"Loaded CSV with {len_dopp_scans} scan entries")
    if len_radar != len_dopp_scans:
        print("Number of radar scans does not match number of Doppler entries in CSV")
        return

    cart_dir = osp.join(output_dir, 'cart')
    if not osp.exists(cart_dir): os.makedirs(cart_dir)
    dopp_est_dir = osp.join(output_dir, 'dopp_est')
    if not osp.exists(dopp_est_dir): os.makedirs(dopp_est_dir)

    # Group dataframe by frame timestamp for faster access
    # { frame_timestamp(int): {"dopp_timestamp": np.array, "dopp_velocity": np.array, "azimuth_idx": np.array} }
    df_by_frame = {}
    for ts, group in df.groupby("frame_timestamp"):
        df_by_frame[int(ts)] = {
            "dopp_timestamp": group["dopp_timestamp"].values,
            "dopp_velocity":  group["dopp_velocity"].values,
            "azimuth_idx":    group["azimuth_idx"].values,
        }

    # Prepare output dirs
    cart_dir = osp.join(output_dir, 'cart');
    os.makedirs(cart_dir, exist_ok=True)
    dopp_est_dir = osp.join(output_dir, 'dopp_est');
    os.makedirs(dopp_est_dir, exist_ok=True)

    # Build task list
    tasks = []
    for img_file in img_list:
        if not img_file.endswith(".png"):
            continue
        img_path = osp.join(radar_dir, img_file)
        timestamp = int(img_file[:-4])  # from filename
        tasks.append((
            img_path, timestamp, radar_res, max_range, cart_resolution,
            df_by_frame, cart_dir, dopp_est_dir
        ))

    # Parallel execution with tqdm
    max_workers = max(1, (os.cpu_count() or 1) // 2)
    with ProcessPoolExecutor(max_workers=max_workers) as ex:
        for _ in tqdm(ex.map(_process_one_frame, tasks), total=len(tasks), desc="Extracting", smoothing=0):
            pass

    # Generate videos
    print("Making videos")
    generate_video(cart_dir, radar_dir, 'radar', False, H=1024, W=1024)
    generate_video(dopp_est_dir, radar_dir, 'dopp_est', True, H=1024, W=1024)


if __name__ == "__main__":
    main()