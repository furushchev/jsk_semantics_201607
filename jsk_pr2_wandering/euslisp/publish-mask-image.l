;; publish-mask-image.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "publish_mask_image")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")

(pr2-init)

(defparameter *mask-image-seq* 0)

(defparameter *scene* (make-eng2-scene))
(defparameter *robot* *pr2*)
(defparameter *cam* (send *robot* :kinect_head-rgb))

(defparameter *object-name* (ros::get-param "~object_name"))
(defparameter *mask-image-topic* (format nil "~A/~A" (ros::get-name) "output"))
(defparameter *mask-image-frame-id* (ros::get-param "~frame_id" "/head_mount_kinect_rgb_optical_frame"))

(unless *object-name*
  (error "you must specify object name")
  (exit 1))

;; init viewer
(send *ri* :objects (list *scene*))
;;(send *cam* :create-viewer)

;; assoc objects / spots
(setq *door-front-spot* (send *scene* :spot "/eng2/7f/room73B2-door-front"))
(setq *fridge* (send *scene* :object "room73b2-hitachi-fiesta-refrigerator"))
(send *fridge* :translate (float-vector 0 500 0))
(setq *table* (send *scene* :object "room73b2-karimoku-table"))
(send *table* :translate (float-vector -200 -200 0))

(if (send *ri* :simulation-modep)
    (progn
      (send *robot* :transform (send *door-front-spot* :worldcoords))
      (send *ri* :go-pos-unsafe 0 0 180))
    (send *ri* :update-robot-state))

(defun create-mask-image (objs)
  (send *pr2* :reset-coords)
  (let ((cds (send *ri* :state :worldcoords)))
    (while (null cds)
           (setq cds (send *ri* :state :worldcoords)))
    (send *pr2* :transform cds :world))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (send *cam* :draw-objects objs)
  (send (send *cam* :get-image) :monochromize))

(defun publish-mask-image ()
  (let* ((mask (create-mask-image (list
                                   (send *scene* :object *object-name*))))
         (msg (ros::image->sensor_msgs/Image mask
                                            :seq (inc *mask-image-seq*)
                                            :frame_id *mask-image-frame-id*)))
    (unless (ros::get-topic-publisher *mask-image-topic*)
      (ros::advertise *mask-image-topic* sensor_msgs::Image 5)
      (unix:sleep 1))
    (ros::publish *mask-image-topic* msg)
    t))

#|
(defun publish-fridge-mask-image ()
  (let* ((mask (create-mask-image (list *fridge*)))
         (msg (ros::image->sensor_msgs/Image mask
                                            :seq (inc *mask-image-seq*)
                                            :frame_id *mask-image-frame-id*)))
    (unless (ros::get-topic-publisher *mask-image-topic*)
      (ros::advertise *mask-image-topic* sensor_msgs::Image 5)
      (unix:sleep 1))
    (ros::publish *mask-image-topic* msg)
    t))
|#

(while (ros::ok)
  (publish-mask-image)
  (send *ri* :spin-once))
