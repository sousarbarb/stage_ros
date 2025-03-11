define hokuyoust10lx ranger
(
  sensor
  (
    range   [ 0.06 10.0 ]         # mininum maximum in meters
    fov     270.0                 # field of view in degrees
    samples 1081                  # number of samples
    pose    [ 0.0 0.0 0.0474 0 ]  # pose of the transducer relative to its parent
  )

  color_rgba [ 0 0 0 1.0 ]
  size       [ 0.05 0.05 0.07 ]
)
