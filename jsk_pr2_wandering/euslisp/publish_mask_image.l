;; publish-mask-image.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")

(defparameter *object-name* (ros::get-param "~object_name"))
(defparameter *mask-image-topic* (format nil "~A/~A" (ros::get-name) "output"))
(defparameter *mask-image-frame-id* (ros::get-param "~frame_id" "/head_mount_kinect_rgb_optical_frame"))
(defparameter *publish-marker-p* (ros::get-param "~publish_marker" nil))
(defparameter *marker-topic* (format nil "~A/~A" (ros::get-name) "marker"))
(unless *object-name*
  (error "you must specify object name")
  (exit 1))

(unless (ros::get-topic-publisher *mask-image-topic*)
  (ros::advertise *mask-image-topic* sensor_msgs::Image 5)
  (unix:sleep 1))
(unless (ros::get-topic-publisher *marker-topic*)
  (ros::advertise *marker-topic* visualization_msgs::Marker 5)
  (unix:sleep 1))

(pr2-init)

(defparameter *mask-image-seq* 0)

(defparameter *scene* (make-eng2-scene))
(defparameter *cam* (send *pr2* :kinect_head-rgb))
(defparameter *marker-id* (ros::get-param "masked_object_marker_id" 0))
(ros::set-param "masked_object_marker_id" (inc *marker-id*))

(when (and (boundp '*compile-message*) *compile-message*)
  (setq ros::*compile-message* t)
  (ros::load-ros-manifest "sensor_msgs")
  (ros::load-ros-manifest "visualization_msgs"))

;; init viewer
(send *ri* :objects (list *scene*))

;; assoc objects / spots
(setq *door-front-spot* (send *scene* :spot "/eng2/7f/room73B2-door-front"))
(setq *fridge* (send *scene* :object "room73b2-hitachi-fiesta-refrigerator"))
(send *fridge* :translate (float-vector 0 500 0))
(setq *table* (send *scene* :object "room73b2-karimoku-table"))
(send *table* :translate (float-vector -400 -150 0))
(setq *drawer* (send *scene* :object "room73b2-hrp2-parts-drawer"))
(send *drawer* :translate (float-vector 0 200 0))

(defun create-mask-image (objs)
  (send *ri* :update-robot-state)
  (send *pr2* :reset-coords)
  (let ((cds (send *ri* :state :worldcoords)))
    (while (null cds)
           (setq cds (send *ri* :state :worldcoords)))
    (send *pr2* :transform cds :world))
  (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
  (ros::ros-info "[~A] robot pose: ~A, joints: ~A"
                 (ros::get-name) (send *pr2* :worldcoords) (send *ri* :state :potentio-vector))
  (send *cam* :draw-objects objs)
  (send (send *cam* :image-viewer :viewsurface :getglimage :offscreen t) :monochromize))

(defun publish-mask-image ()
  (let* ((obj (send *scene* :object *object-name*))
         (mask (create-mask-image (list obj)))
         msg)
    (when (> (send mask :average-pixel) 0.1)
      (setq msg (ros::image->sensor_msgs/Image mask
                                               :seq (inc *mask-image-seq*)
                                               :frame_id *mask-image-frame-id*))
      (ros::publish *mask-image-topic* msg))
    (when *publish-marker-p*
      (ros::publish *marker-topic*
                    (object->marker-msg obj
                                        (instance std_msgs::header :init
                                                  :stamp (ros::time-now)
                                                  :frame_id "world")
                                        :ns "masked_objects"
                                        :lifetime 300
                                        :id *marker-id*)))
    t))

#|
;; for debug
(defun publish-mask-image ()
  (let* ((mask (create-mask-image (list *fridge*)))
         (msg (ros::image->sensor_msgs/Image mask
                                            :seq (inc *mask-image-seq*)
                                            :frame_id *mask-image-frame-id*)))
    (unless (ros::get-topic-publisher *mask-image-topic*)
      (ros::advertise *mask-image-topic* sensor_msgs::Image 5)
      (unix:sleep 1))
    (ros::publish *mask-image-topic* msg)
    (objects (list *pr2* *fridge*))
    t))
|#
