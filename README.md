loop_closing
=============

ROS library for HAsh-based LOop Closure. This library provides the tools for image loop closing based on image hashing. Image hashing consists of representing every image with a small vector (hash). Then the hash of image A can be compared with the hash of image B in a super fast way in order to determine if images are similar.

The image hashing implemented in this library is based on features (SIFT, SURF, ORB, etc.) so, if the features you choose are invariant to scale and rotation, the image hashing will be also invariant to these properties.

The library works for both mono and stereo datasets and provides a transformation (2d for mono and 3d for stereo) when loop closures are found.

How it works
-------

Please see the examples directory for functional implementations. You can also check [this][stereo_slam] integration for a 3D Stereo Slam.


1) Declare your lc object like this:
```bash
lc::LoopClosure lc_obj;
```

2) Set the lc parameters:
```bash
lc::LoopClosure::Params params;
params.work_dir = whatever;
params.desc_type = whatever;
.
.
.
params.validate = whatever;
lc_obj.setParams(params);
```

3) Initialize the object
```bash
lc_obj.init();
```

4) Then, for every image, call to setNode to save the new image and getLoopClosure to get any possible loop closure between the last image and any previous images.
```bash
// Mono version
int loop_closure_with; 	// <- Will contain the index of the image that closes loop with the last inserted (-1 if none).
lc_obj.setNode(img);
bool valid = lc_obj.getLoopClosure(loop_closure_with);

// Stereo version
int loop_closure_with; 	// <- Will contain the index of the image that closes loop with the last inserted (-1 if none).
tf::Transform trans; 	// <- Will contain the transformation of the loop closure (if any).
lc_obj.setNode(img_left, img_right);
bool valid = lc_obj.getLoopClosure(loop_closure_with, trans);
```

In both cases, if valid is true, then a loop closure has been found, false otherwise.


Most Important Parameters
-------

* `work_dir` - Directory where the library will save the image informations (must be writtible!).
* `desc_type` - Type of the descriptors (can be SIFT, SURF).
* `desc_thresh` - Descriptor threshold (tipically between 0.7-0.9).
* `min_neighbour` - Minimum number of neighbours that will be skiped for the loop closure (tipically between 5-20, but depends on the frame rate).
* `n_candidates` - Get the n first candidates of the hash matching (tipically between 1-5).
* `min_matches` - Minimun number of descriptor matches to consider a matching as possible loop closure (>8).
* `min_inliers` - Minimum number of inliers to consider a matching as possible loop closure (>8).

Demo
-------

See it in action [here][link_demo].


[link_demo]: http://pul.uib.es/loop_closing/
[stereo_slam]: https://github.com/srv/stereo_slam
