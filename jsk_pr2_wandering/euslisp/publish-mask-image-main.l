#!/usr/bin/env roseus
;; publish-mask-image.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "publish_mask_image")

(setq ros::*compile-message* t)
(ros::load-ros-manifest "sensor_msgs")
(ros::load-ros-manifest "visualization_msgs")
(setq ros::*compile-message* nil) ;; temp workaround

(let ((path (ros::resolve-ros-path "package://jsk_pr2_wandering/euslisp/publish_mask_image")))
  (compiler::compile-file-if-src-newer path path)
  (require :publish_mask_image path)

(ros::ros-info "[~A] publishing mask of object: ~A -> ~A"
               (ros::get-name) *object-name* *mask-image-topic*)
(while (ros::ok)
  (publish-mask-image))
(exit 0)
