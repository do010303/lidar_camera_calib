"""Evaluate a FIXED transform on held-out board samples; never optimize it."""
import argparse
import csv
import hashlib
from pathlib import Path
import numpy as np
import yaml
from .geometry import load_camera


def metrics(errors):
    errors = np.asarray(errors, float)
    if not errors.size or not np.isfinite(errors).all():
        raise ValueError('Empty or nonfinite errors')
    return dict(rmse_m=float(np.sqrt(np.mean(errors**2))),
                mean_signed_m=float(np.mean(errors)),
                mean_absolute_m=float(np.mean(np.abs(errors))),
                p95_absolute_m=float(np.percentile(np.abs(errors),95)),
                max_absolute_m=float(np.max(np.abs(errors))))


def evaluate(data_dir, extrinsic_path, camera_path, training_dir=None):
    with Path(extrinsic_path).expanduser().open() as f:
        ext = yaml.safe_load(f)
    T = np.asarray(ext['transform_4x4'],float)
    if T.shape != (4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-6) or not np.isclose(np.linalg.det(T[:3,:3]),1):
        raise ValueError('Invalid rigid transform')
    cfg,K,D = load_camera(camera_path)
    expected = np.r_[K.ravel(),D,cfg['board_cols'],cfg['board_rows'],cfg['square_size_m'],cfg['image_width'],cfg['image_height']]
    paths = sorted(Path(data_dir).expanduser().glob('sample_*.npz'))
    if not paths:
        raise ValueError('No sample_*.npz files found')
    training_stamps = set()
    if training_dir:
        for path in Path(training_dir).expanduser().glob('sample_*.npz'):
            with np.load(path,allow_pickle=False) as d:
                training_stamps.add((float(d['stamp_image']),float(d['stamp_scan'])))
    rows=[]; errors=[]; normals=[]; seen=set(); duplicate_count=0
    for path in paths:
        with np.load(path,allow_pickle=False) as d:
            p=np.asarray(d['points_l'],float); n=np.asarray(d['normal_c'],float); offset=float(d['offset_c'])
            if p.ndim!=2 or p.shape[1]!=3 or len(p)<6 or n.shape!=(3,) or not np.isfinite(p).all() or not np.isfinite(n).all() or not np.isfinite(offset) or not np.isclose(np.linalg.norm(n),1) or not np.allclose(p[:,2],0):
                raise ValueError(f'Invalid geometry: {path.name}')
            if str(d['source_frame'].item())!=ext['source_frame'] or str(d['target_frame'].item())!=ext['target_frame']:
                raise ValueError(f'Frame mismatch: {path.name}')
            signature=np.r_[d['K'].ravel(),d['D'].ravel(),d['board_size'],d['square_size_m'],d['image_size']]
            if signature.shape!=expected.shape or not np.allclose(signature,expected,rtol=1e-9,atol=1e-12):
                raise ValueError(f'Intrinsic/board mismatch: {path.name}')
            stamps=(float(d['stamp_image']),float(d['stamp_scan']))
            if stamps in training_stamps:
                raise ValueError(f'Training/check data overlap: {path.name}')
            duplicate_count += int(stamps in seen); seen.add(stamps)
            pc=(T[:3,:3]@p.T).T+T[:3,3]
            e=pc@n+offset
            row=dict(sample=path.name,n_points=len(p),**metrics(e),
                     timestamp_delta_s=abs(stamps[0]-stamps[1]),
                     fraction_behind_camera=float(np.mean(pc[:,2]<=0)))
            rows.append(row); errors.append(e); normals.append(n)
    pooled=np.concatenate(errors)
    singular=np.linalg.svd(np.stack(normals),compute_uv=False)
    warnings=[]
    if duplicate_count: warnings.append(f'{duplicate_count} repeated image/scan timestamp pairs; samples are not all independent')
    if len(singular)<3 or singular[-1]<singular[0]*.01: warnings.append('Board normals have limited diversity; test more tilt directions')
    if any(r['fraction_behind_camera'] for r in rows): warnings.append('Some selected points are behind camera')
    if max(r['timestamp_delta_s'] for r in rows)>.1: warnings.append('Some timestamp deltas exceed 0.1 seconds')
    report=dict(extrinsic=str(Path(extrinsic_path).resolve()),
        extrinsic_sha256=hashlib.sha256(Path(extrinsic_path).read_bytes()).hexdigest(),
        data_dir=str(Path(data_dir).resolve()),n_samples=len(rows),n_points=len(pooled),
        metric='Fixed-transform point-to-board-plane distance, not 3D tip position or pixel error',
        point_weighted=metrics(pooled),
        equal_sample_rmse_m=float(np.sqrt(np.mean([r['rmse_m']**2 for r in rows]))),
        training_rmse_m=ext.get('point_plane_rmse_m'),
        normal_singular_values=singular.tolist(),duplicate_timestamp_pairs=duplicate_count,
        training_overlap_checked=bool(training_dir),warnings=warnings,
        worst_samples=sorted(rows,key=lambda r:r['rmse_m'],reverse=True)[:5])
    return report,rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir',default='samples_check')
    parser.add_argument('--extrinsic',default='extrinsic.yaml')
    parser.add_argument('--camera-yaml',default='config/camera.yaml')
    parser.add_argument('--training-dir',help='Original training samples; reject overlapping timestamp pairs')
    parser.add_argument('--output',default='evaluation.yaml')
    args=parser.parse_args()
    output=Path(args.output).expanduser(); csv_path=output.with_suffix('.csv')
    protected=[Path(args.extrinsic).expanduser().resolve(),Path(args.camera_yaml).expanduser().resolve()]
    if output.resolve() in protected or csv_path.resolve() in protected:
        parser.error('Output must not overwrite extrinsic or camera configuration')
    try:
        report,rows=evaluate(args.data_dir,args.extrinsic,args.camera_yaml,args.training_dir)
    except (ValueError,KeyError,OSError) as e:
        parser.exit(1,f'Evaluation failed: {e}\n')
    with output.open('w') as f: yaml.safe_dump(report,f,sort_keys=False,allow_unicode=True)
    with csv_path.open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)
    print(f"Samples: {report['n_samples']}, points: {report['n_points']}")
    for key,value in report['point_weighted'].items(): print(f'{key}: {value*1000:.3f} mm')
    print(f"Equal-sample RMSE: {report['equal_sample_rmse_m']*1000:.3f} mm")
    for w in report['warnings']: print('WARNING:',w)
    print('Worst samples:')
    for row in report['worst_samples']: print(f"  {row['sample']}: {row['rmse_m']*1000:.3f} mm")
    print('Reports:',output,csv_path,'; extrinsic unchanged')


if __name__=='__main__': main()
