"""Read-only mesh oracle and metric checks, independent of detector output."""
from pathlib import Path
import sys
import numpy as np
WS=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(WS/'tools'))
from d05_groundtruth import load_centerlines, distances, sample_segments, named_poses, project_ground


def test_mesh_is_two_centimetre_strips():
    segments=load_centerlines(WS)
    assert segments.shape==(403,2,3)
    length=np.linalg.norm(segments[:,1,:2]-segments[:,0,:2],axis=1).sum()
    assert 15.<length<17.
    assert distances(np.array([[.50,.33],[.50,.35]]),segments).tolist()==\
        __import__('pytest').approx([0.,.02],abs=1e-6)


def test_sampling_and_named_oracle_poses():
    segments=load_centerlines(WS)
    samples=sample_segments(segments,.005)
    assert len(samples)>3000
    assert np.max(distances(samples[:,:2],segments))<1e-6
    level=project_ground(np.array([[.6,0.]]))
    pitched=project_ground(np.array([[.6,0.]]),pitch=-.005)
    assert abs(level[0,0]-319.5)<1e-9
    assert pitched[0,1]>level[0,1]
    poses=named_poses(segments)
    assert set(poses)=={'start_t','straight','corner','curve','crossing','finish_t','hill_entry'}
