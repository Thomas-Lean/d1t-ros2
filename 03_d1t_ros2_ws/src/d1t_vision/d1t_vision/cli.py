"""Offline commands; --help works without ROS2 or OpenCV."""
import argparse
from pathlib import Path
import sys
from .board import detect_board, render_board
from .calibration import check_held_out, load_calibration, sample_point, solve_dataset
from .geometry import ValidationError
from .runtime import validate_config
from .samples import load_sample, read_json, sample_paths, write_new_json, json_bytes


def main(argv=None):
    parser=argparse.ArgumentParser(description='D1-T read-only eye-in-hand tools; metres/radians, no motion')
    sub=parser.add_subparsers(dest='command',required=True)
    c=sub.add_parser('check-config',help='strictly validate completed runtime JSON')
    c.add_argument('config',type=Path)
    b=sub.add_parser('board',help='render ChArUco PNG from board JSON; verify physical print separately')
    b.add_argument('board_spec',type=Path)
    b.add_argument('output_png',type=Path)
    b.add_argument('--pixels-per-square',type=int,default=160)
    s=sub.add_parser('solve',help='solve >=8 complete same-session sample bundles')
    s.add_argument('dataset',type=Path)
    s.add_argument('output',type=Path)
    s.add_argument('--max-translation-residual-m',type=float,required=True)
    s.add_argument('--max-rotation-residual-rad',type=float,required=True)
    d=sub.add_parser('detect',help='redetect a saved color image; never overwrite sample observation')
    d.add_argument('sample',type=Path)
    p=sub.add_parser('point',help='saved aligned-depth pixel -> optical point -> base_link')
    p.add_argument('sample',type=Path)
    p.add_argument('calibration',type=Path)
    p.add_argument('u',type=float)
    p.add_argument('v',type=float)
    p.add_argument('--allow-test-data',action='store_true',help='offline synthetic data only')
    h=sub.add_parser('check-held-out',help='new observations: cross-pose spread and optional independent survey error')
    h.add_argument('dataset',type=Path)
    h.add_argument('calibration',type=Path)
    h.add_argument('output',type=Path)
    h.add_argument('--corner-ids',type=int,nargs='+',required=True)
    h.add_argument('--independent-reference',type=Path)
    h.add_argument('--allow-test-data',action='store_true')
    args=parser.parse_args(argv)
    try:
        if args.command=='check-config':
            validate_config(read_json(args.config))
            result={'config_schema':'valid','hardware':'未验证; configuration validation is not a hardware observation'}
        elif args.command=='board':
            cv2,img=render_board(read_json(args.board_spec),args.pixels_per_square)
            ok,encoded=cv2.imencode('.png',img)
            if not ok: raise ValidationError('PNG encoding failed')
            with args.output_png.open('xb') as f: f.write(encoded.tobytes())
            result={'design_png':str(args.output_png),'printing':'Set physical size from nominal_square_length_m; measure after printing. PNG pixels do not guarantee print scale.'}
        elif args.command=='solve':
            result=solve_dataset(sample_paths(args.dataset),args.output,args.max_translation_residual_m,args.max_rotation_residual_rad)
        elif args.command=='detect':
            m,color,_=load_sample(args.sample)
            result=detect_board(color,m['color']['encoding'],m['color_info'],m['image_geometry'],m['board'],m['observation']['max_reprojection_rms_px'])
        elif args.command=='point':
            m,_,depth=load_sample(args.sample)
            result=sample_point(m,depth,load_calibration(args.calibration,args.allow_test_data),[args.u,args.v])
        else:
            calibration=load_calibration(args.calibration,args.allow_test_data)
            reference=read_json(args.independent_reference) if args.independent_reference else None
            result=check_held_out(sample_paths(args.dataset),calibration,args.corner_ids,reference)
            write_new_json(args.output,result)
        print(json_bytes(result).decode(),end='')
        return 0
    except (ValidationError,OSError,KeyError,ValueError) as exc:
        print(f'VISION_ERROR: {exc}',file=sys.stderr)
        return 2


if __name__=='__main__':
    sys.exit(main())
