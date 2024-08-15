import numpy as np
import pandas as pd

def sample_robot_region(
        n=1000,
        region_center=[0, -0.05, 0.16],
        region_size=[0.12, 0.1, 0.06],
        robot_length=0.2,
        z_axis_buffer=0.03):
    '''Sample within a region of interest in the tendon robot's workspace
    Uses rejection sampling to sample within the region, and reject if the
    sample is outside of the robot workspace sphere or inside of the cylinder
    about the z-axis with radius z_axis_buffer.

    Returns a nx3 vector of x,y,z points

    Example:
    >>> vals = sample_robot_region(n=100)
    >>> assert vals.shape == (3, 100)
    >>> x = vals[:,0]
    >>> y = vals[:,1]
    >>> z = vals[:,2]
    '''
    region_center = np.asarray(region_center)
    region_size = np.asarray(region_size)
    assert len(region_center) == 3
    assert len(region_size) == 3

    # sample many more than needed, 10*n in this case
    # so that after rejection, we have n samples (hopefully)
    vals = np.random.uniform(low=region_center - region_size/2,
                             high=region_center + region_size/2,
                             size=(10*n, 3))
    is_in_sphere = (np.linalg.norm(vals, axis=1) <= robot_length)
    is_in_zbuf   = (np.linalg.norm(vals[:,:2], axis=1) <= z_axis_buffer)
    # filter out rejections
    to_keep = vals[is_in_sphere & ~is_in_zbuf, :]
    assert to_keep.shape[0] >= n
    return to_keep[:n, :]

samples = sample_robot_region()
df = pd.DataFrame(samples, columns=['x', 'y', 'z'])
df.index.name = 'index'
fname = 'samples.csv'
print(f'writing {len(df)} samples to {fname}')
df.to_csv(fname)
