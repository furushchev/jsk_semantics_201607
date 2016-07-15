;; publish-mask-image.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "publish_mask_image")

(setq *compile-message* t)
(require :publish-mask-image "publish_mask_image")

(ros::ros-info "[~A] publishing mask of object: ~A -> ~A"
               (ros::get-name) *object-name* *mask-image-topic*)
(while (ros::ok)
  (publish-mask-image))
(exit 0)
