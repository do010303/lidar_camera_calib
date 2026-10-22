"""Offline point-plane calibration: p_camera = R @ p_lidar + t."""
import argparse
from pathlib import Path
import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares


def residual(x, samples):
    R = cv2.Rodrigues(np.asarray(x[:3],float))[0]; t=np.asarray(x[3:])
    return np.concatenate([s['normal_c'] @ (R@s['points_l'].T+t[:,None])+s['offset_c'] for s in samples])


def linear_initial(samples):
    # z_lidar=0: estimate first two rotation columns and translation.
    A=[]; b=[]
    for s in samples:
        p=s['points_l']; n=s['normal_c']; weight=1/np.sqrt(len(p))
        A.append(np.c_[p[:,0,None]*n,p[:,1,None]*n,np.tile(n,(len(p),1))]*weight)
        b.append(np.full(len(p),-s['offset_c'])*weight)
    A=np.vstack(A); b=np.concatenate(b)
    v=np.linalg.svd(A,compute_uv=False)
    if v[-1] < v[0]*1e-6:
        raise ValueError('Linear initialization ill-conditioned; collect diverse board tilts or supply measured --init')
    x=np.linalg.lstsq(A,b,rcond=None)[0]
    basis=np.column_stack((x[:3],x[3:6]))
    u,_,vh=np.linalg.svd(basis,full_matrices=False)
    basis=u@vh
    R=np.column_stack((basis,np.cross(basis[:,0],basis[:,1])))
    return np.r_[cv2.Rodrigues(R)[0].ravel(),x[6:]]


def camera_pose(r, c):
    """Rotation and t for rotation vector r and camera position c=(fwd,left,up) in the LiDAR frame.

    A planar scan never uses R[:,2], so its sign is unobservable; 'up' is taken as the column pointing up in the
    camera image (negative optical y), which also fixes the sign of the camera's height above the scan plane.
    """
    R = cv2.Rodrigues(np.asarray(r, float))[0]
    up = R[:, 2] if R[1, 2] < 0 else -R[:, 2]
    return R, -(R[:, 0]*c[0]+R[:, 1]*c[1]+up*c[2])


def optimize(samples, initial):
    def balanced(x):
        raw=residual(x,samples); start=0; parts=[]
        for s in samples:
            count=len(s['points_l']); parts.append(raw[start:start+count]/np.sqrt(count)); start+=count
        return np.concatenate(parts)
    result=least_squares(balanced,initial,loss='soft_l1',f_scale=.003,x_scale='jac',max_nfev=5000)
    # Numerical Jacobian in local physical coordinates: rotation radians, t metres.
    eps=1e-6
    J=np.column_stack([(balanced(result.x+np.eye(6)[i]*eps)-balanced(result.x-np.eye(6)[i]*eps))/(2*eps) for i in range(6)])
    singular=np.linalg.svd(J,compute_uv=False)
    rank=int(np.sum(singular > singular[0]*1e-5)) if singular[0] else 0
    condition=float(singular[0]/singular[-1]) if singular[-1] else float('inf')
    return result, singular, rank, condition


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--data-dir',default='samples')
    parser.add_argument('--output',default='extrinsic.yaml')
    parser.add_argument('--init',nargs=6,type=float,metavar=('RX','RY','RZ','TX','TY','TZ'),help='Measured approximate Rodrigues radians + translation metres; default uses data-derived linear initialization')
    parser.add_argument('--max-rmse',type=float,default=.02)
    parser.add_argument('--camera-pos',nargs=3,type=float,metavar=('FWD','LEFT','UP'),help='Measured camera position in the LiDAR frame (m, relative to scan plane centre); fixes translation and solves rotation only')
    parser.add_argument('--no-reject',action='store_true',help='Keep every pose; default drops outlier poses (wrong ROI, motion) one at a time')
    parser.add_argument('--min-poses',type=int,default=12,help='Stop rejecting outliers at this many poses')
    args=parser.parse_args()
    samples=[]; frames=None; camera=None
    for path in sorted(Path(args.data_dir).expanduser().glob('sample_*.npz')):
        with np.load(path,allow_pickle=False) as d:
            s={k:d[k].copy() for k in ('points_l','normal_c','offset_c')}
            pair=tuple(str(d[k].item()) for k in ('source_frame','target_frame'))
            signature=np.r_[d['K'].ravel(),d['D'].ravel(),d['board_size'],d['square_size_m'],d['image_size']]
        p=s['points_l']; n=s['normal_c']
        if p.ndim!=2 or p.shape[1]!=3 or len(p)<6 or n.shape!=(3,) or not all(np.isfinite(v).all() for v in s.values()) or not np.isclose(np.linalg.norm(n),1) or not np.allclose(p[:,2],0):
            raise SystemExit(f'Invalid sample: {path}')
        if frames is not None and (pair!=frames or signature.shape!=camera.shape or not np.allclose(signature,camera)):
            raise SystemExit('Mixed frame IDs, intrinsic or board settings in sample directory')
        frames=pair; camera=signature; s['name']=path.name; samples.append(s)
    if len(samples)<8: raise SystemExit('Need >=8 poses; recommend 20-30 diverse poses')
    normals=np.stack([s['normal_c'] for s in samples])
    ns=np.linalg.svd(normals,compute_uv=False)
    if ns[-1]<ns[0]*.01: raise SystemExit('Board normals insufficiently diverse: translation is unobservable/weak')
    try:
        initial=np.asarray(args.init,float) if args.init is not None else linear_initial(samples)
        result,singular,rank,condition=optimize(samples,initial)
    except ValueError as e:
        raise SystemExit(str(e))
    dropped=[]
    while not args.no_reject and len(samples)>args.min_poses:
        errs=[float(np.sqrt(np.mean(residual(result.x,[s])**2))) for s in samples]
        worst=int(np.argmax(errs))
        if errs[worst]<=max(2.5*float(np.median(errs)),.015): break
        dropped.append((samples[worst]['name'],errs[worst])); del samples[worst]
        result,singular,rank,condition=optimize(samples,result.x)
    if args.camera_pos:
        c=np.asarray(args.camera_pos,float)
        def balanced_c(r):
            x=np.r_[r,camera_pose(r,c)[1]]
            return np.concatenate([residual(x,[s])/np.sqrt(len(s['points_l'])) for s in samples])
        fit=least_squares(balanced_c,result.x[:3],loss='soft_l1',f_scale=.003,x_scale='jac',max_nfev=5000)
        eps=1e-6
        J=np.column_stack([(balanced_c(fit.x+np.eye(3)[i]*eps)-balanced_c(fit.x-np.eye(3)[i]*eps))/(2*eps) for i in range(3)])
        singular=np.linalg.svd(J,compute_uv=False); rank=int(np.sum(singular>singular[0]*1e-5)); condition=float(singular[0]/singular[-1])
        result.x=np.r_[fit.x,camera_pose(fit.x,c)[1]]; result.success=fit.success
        if rank<3: raise SystemExit('Rotation unobservable even with fixed camera position')
        print('Camera position fixed at (fwd,left,up) m:',c.tolist(),'- rotation only')
    for name,err in dropped: print(f'Rejected outlier pose {name}: RMSE {err*1000:.0f} mm')
    R=cv2.Rodrigues(result.x[:3])[0]; t=result.x[3:]
    per_pose=[float(np.sqrt(np.mean(residual(result.x,[s])**2))) for s in samples]
    rms=float(np.sqrt(np.mean(residual(result.x,samples)**2)))
    front=min(float(np.mean(((R@s['points_l'].T).T+t)[:,2]>0)) for s in samples)
    print('Initial [rvec,t]:',initial)
    print('success:',result.success,'rank:',rank,'condition:',condition,'RMSE [m]:',rms)
    print('Per-pose RMSE [m]:',per_pose)
    if not result.success or rank<(3 if args.camera_pos else 6) or condition>1e5 or rms>args.max_rmse or front<1:
        raise SystemExit('Calibration rejected: convergence/observability/residual/cheirality failed. No output written.')
    T=np.eye(4); T[:3,:3]=R; T[:3,3]=t
    out=dict(source_frame=frames[0],target_frame=frames[1],convention='p_camera_optical = R @ p_lidar + t',
        rotation_matrix=R.tolist(),translation_m=t.tolist(),transform_4x4=T.tolist(),
        point_plane_rmse_m=rms,per_pose_rmse_m=per_pose,jacobian_rank=rank,
        jacobian_singular_values=singular.tolist(),jacobian_condition=condition,n_poses=len(samples),
        validation='Numerical checks passed; independent real sensor overlay still required')
    with Path(args.output).expanduser().open('w') as f: yaml.safe_dump(out,f,sort_keys=False)
    print('Saved T_camera_lidar:',args.output)


if __name__=='__main__': main()
