#!/usr/bin/env roseus
;; publish-object-bounding-box-main.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


(ros::roseus "publish_object_bounding_box")

(setq ros::*compile-message* t)
(ros::load-ros-manifest "visualization_msgs")
(ros::load-ros-manifest "jsk_recognition_msgs")
(setq ros::*compile-message* nil) ;; temp workaround

(let ((path (ros::resolve-ros-path "package://jsk_pr2_wandering/euslisp/publish_object_bounding_box")))
  (compiler::compile-file-if-src-newer path path)
  (require :publish_object_bounding_box path))

(publish)
