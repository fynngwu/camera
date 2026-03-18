#!/usr/bin/env python3
"""Command-line interface for camera calibration toolkit.

Usage:
    uv run capture              # Capture from USB camera
    uv run capture -u           # Capture from default RTSP stream
    uv run capture -u rtsp://... # Capture from custom RTSP stream
    uv run calibrate            # Run calibration
    uv run undistort            # Live undistort demo
    uv run undistort --debug    # Debug mode with matrix logging
"""

import argparse
import sys


def create_parser() -> argparse.ArgumentParser:
    """Create the argument parser with subcommands."""
    parser = argparse.ArgumentParser(
        prog='camera-cal',
        description='Camera calibration toolkit'
    )
    subparsers = parser.add_subparsers(dest='command', required=True, help='Available commands')

    # capture subcommand
    capture_parser = subparsers.add_parser('capture', help='Capture calibration images')
    capture_parser.add_argument(
        '--camera',
        type=int,
        default=0,
        help='Camera device ID (default: 0 for /dev/video0)'
    )
    capture_parser.add_argument(
        '--output',
        type=str,
        default='calibration_images',
        help='Output directory for captured images'
    )
    capture_parser.add_argument(
        '-u', '--url',
        type=str,
        default='rtsp://100.103.8.66:8554/video',
        help='RTSP stream URL (default: rtsp://100.103.8.66:8554/video). Use --usb to use USB camera instead.'
    )
    capture_parser.add_argument(
        '--usb',
        action='store_true',
        help='Force use USB camera instead of RTSP stream'
    )
    capture_parser.set_defaults(func='capture')

    # calibrate subcommand
    calibrate_parser = subparsers.add_parser('calibrate', help='Calibrate camera using chessboard images')
    calibrate_parser.add_argument(
        '--images',
        type=str,
        default='calibration_images',
        help='Directory containing calibration images'
    )
    calibrate_parser.add_argument(
        '--output',
        type=str,
        default='calibration_data',
        help='Output directory for calibration results'
    )
    calibrate_parser.add_argument(
        '--square-size',
        type=float,
        default=30.0,
        help='Actual size of chessboard square in millimeters'
    )
    calibrate_parser.set_defaults(func='calibrate')

    # undistort subcommand
    undistort_parser = subparsers.add_parser('undistort', help='Demonstrate camera distortion correction')
    undistort_parser.add_argument(
        '--camera',
        type=int,
        default=0,
        help='Camera device ID for live demo (default: 0)'
    )
    undistort_parser.add_argument(
        '--calibration',
        type=str,
        default='calibration_data/camera_calibration.npz',
        help='Path to calibration file'
    )
    undistort_parser.add_argument(
        '--mode',
        type=str,
        choices=['live', 'images', 'rtsp'],
        default='rtsp',
        help='Demo mode: live camera, saved images, or RTSP stream (default: rtsp)'
    )
    undistort_parser.add_argument(
        '--images-dir',
        type=str,
        default='calibration_images',
        help='Directory containing images to process (for --mode images)'
    )
    undistort_parser.add_argument(
        '--output-dir',
        type=str,
        default='calibration_data',
        help='Output directory for comparison images'
    )
    undistort_parser.add_argument(
        '--url',
        type=str,
        default='rtsp://100.103.8.66:8554/video',
        help='RTSP stream URL (for --mode rtsp)'
    )
    undistort_parser.add_argument(
        '--debug',
        action='store_true',
        help='Enable debug mode to log camera matrix and distortion coefficients'
    )
    undistort_parser.set_defaults(func='undistort')

    return parser


def main():
    """Main entry point for the CLI."""
    parser = create_parser()
    args = parser.parse_args()

    if args.func == 'capture':
        from camera_cal.capture import capture_images
        # Use RTSP by default, but use USB camera if --usb flag is provided
        rtsp_url = None if args.usb else args.url
        capture_images(
            camera_id=args.camera,
            output_dir=args.output,
            rtsp_url=rtsp_url,
        )
    elif args.func == 'calibrate':
        from camera_cal.calibrate import calibrate_camera
        success, _ = calibrate_camera(
            images_dir=args.images,
            output_dir=args.output,
            square_size=args.square_size,
        )
        if not success:
            sys.exit(1)
    elif args.func == 'undistort':
        from camera_cal.undistort import main as undistort_main
        import sys
        sys.argv = ['undistort']
        sys.argv.extend(['--mode', args.mode])
        if args.camera != 0:
            sys.argv.extend(['--camera', str(args.camera)])
        if args.calibration != 'calibration_data/camera_calibration.npz':
            sys.argv.extend(['--calibration', args.calibration])
        if args.images_dir != 'calibration_images':
            sys.argv.extend(['--images-dir', args.images_dir])
        if args.output_dir != 'calibration_data':
            sys.argv.extend(['--output-dir', args.output_dir])
        if args.url != 'rtsp://100.103.8.66:8554/video':
            sys.argv.extend(['--url', args.url])
        if args.debug:
            sys.argv.append('--debug')
        undistort_main()


if __name__ == '__main__':
    main()
