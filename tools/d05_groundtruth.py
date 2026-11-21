"""Read-only official STL strip oracle. Never imported by a rover package.

The asset consists of consecutive pairs of triangles forming 2 cm strips.
Each strip's two short boundary edges provide its exact centerline endpoints.
World/model/link visual poses are applied rather than assuming a unit transform.
"""
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np


def transform(pose):
    import math
    x,y,z,r,p,yaw=pose
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(yaw),math.sin(yaw)
    rotation=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                       [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
    matrix=np.eye(4); matrix[:3,:3]=rotation; matrix[:3,3]=[x,y,z]
    return matrix


def pose(element):
    value=element.findtext('pose','0 0 0 0 0 0')
    return transform([float(x) for x in value.split()])


def load_centerlines(workspace):
    share=Path(workspace)/'src/line_follower_bringup'
    world=ET.parse(share/'worlds/line_track.sdf').getroot()
    track=next(i for i in world.find('world').findall('include') if i.findtext('name')=='track')
    model=ET.parse(share/'models/track/model.sdf').getroot().find('model')
    link=model.find("link[@name='line']")
    visual=link.find('visual')
    matrix=pose(track)@pose(model)@pose(link)@pose(visual)
    raw=(share/'models/track/meshes/track.stl').read_bytes()
    count=struct.unpack_from('<I',raw,80)[0]
    assert len(raw)==84+count*50 and count%2==0
    triangles=np.frombuffer(raw,dtype=[('normal','<f4',3),('vertices','<f4',(3,3)),('attribute','<u2')],offset=84,count=count)['vertices'].astype(float)
    result=[]
    for first,second in zip(triangles[::2],triangles[1::2]):
        assert np.allclose(first[[0,2]],second[[0,1]])
        vertices=np.array([*first,second[2]])
        lengths=np.linalg.norm(np.roll(vertices,-1,axis=0)[:,:2]-vertices[:,:2],axis=1)
        # For tight arc tessellation the long direction may be shorter than 2 cm.
        # The radial strip width remains 2 cm, so choose the opposite pair nearest 2 cm.
        scores=[abs(lengths[i]-.02)+abs(lengths[(i+2)%4]-.02) for i in (0,1)]
        i=int(np.argmin(scores))
        assert abs(lengths[i]-.02)<.001 and abs(lengths[(i+2)%4]-.02)<.001
        endpoints=np.array([(vertices[i]+vertices[(i+1)%4])/2,
                            (vertices[(i+2)%4]+vertices[(i+3)%4])/2])
        result.append((np.c_[endpoints,np.ones(2)]@matrix.T)[:,:3])
    return np.array(result)


def distances(points,segments):
    points=np.asarray(points)
    if len(points)==0: return np.empty(0)
    a=segments[:,0,:2]; delta=segments[:,1,:2]-a
    t=np.clip(((points[:,None,:2]-a)*delta).sum(axis=2)/(delta*delta).sum(axis=1),0,1)
    return np.sqrt(((points[:,None,:2]-a-t[:,:,None]*delta)**2).sum(axis=2)).min(axis=1)


def sample_segments(segments,spacing=.005):
    return np.concatenate([np.linspace(a,b,max(2,int(np.linalg.norm(b-a)/spacing)+1)) for a,b in segments])


def named_poses(segments):
    # Indices identify mesh strips, not rover code waypoints. All positions are
    # derived from immutable asset endpoints; offsets put landmarks in the camera ROI.
    start=segments[0,1]; finish=segments[-1].mean(axis=0)
    corner=segments[99,1]; crossing=np.array([segments[320,0,0],segments[143,0,1],.001])
    straight=segments[139].mean(axis=0)
    curve=segments[12,0]
    hill=segments[3,1]
    return {'start_t':(start[0],start[1]+.52,.01,-np.pi/2),
            'straight':(straight[0],straight[1],.01,0.),
            'corner':(corner[0],corner[1]-.52,.01,np.pi/2),
            'curve':(curve[0]-.48,curve[1],.01,0.),
            'crossing':(crossing[0],crossing[1]-.52,.01,np.pi/2),
            'finish_t':(finish[0]+.52,finish[1],.01,np.pi),
            'hill_entry':(hill[0]+.10,hill[1],.03,0.)}


def project_ground(points,pitch=0.,roll=0.):
    """Ground XY to capture pixels, including the small measured camera attitude."""
    points=np.asarray(points)
    f=320/np.tan(62.2*np.pi/360)
    cp,sp,cr,sr=np.cos(pitch),np.sin(pitch),np.cos(roll),np.sin(roll)
    dx=points[:,0]-.076
    rx=cp*dx+sp*.103
    z=sp*dx-cp*.103
    ry=cr*points[:,1]+sr*z
    rz=-sr*points[:,1]+cr*z
    with np.errstate(divide='ignore',invalid='ignore'):
        return np.c_[319.5-f*ry/rx,239.5-f*rz/rx]
