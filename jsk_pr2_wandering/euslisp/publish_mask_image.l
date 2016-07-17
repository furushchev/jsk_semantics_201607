;; publish-mask-image.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :scene "package://jsk_pr2_wandering/euslisp/scene")

(defparameter *object-name* (ros::get-param "~object_name"))
(defparameter *mask-image-topic* (format nil "~A/~A" (ros::get-name) "output"))
(defparameter *mask-image-frame-id* (ros::get-param "~frame_id" "/head_mount_kinect_rgb_optical_frame"))
(defparameter *publish-marker-p* (ros::get-param "~publish_marker" nil))
(defparameter *marker-topic* (format nil "~A/~A" (ros::get-name) "marker"))
(unless *object-name*
  (error "you must specify object name")
  (exit 1))

(unless (ros::get-topic-publisher *mask-image-topic*)
  (ros::advertise *mask-image-topic* sensor_msgs::Image 1)
  (unix:sleep 1))
(when (and *publish-marker-p* (null (ros::get-topic-publisher *marker-topic*)))
  (ros::advertise *marker-topic* visualization_msgs::Marker 1)
  (unix:sleep 1))

(pr2-init)

(defparameter *mask-image-seq* 0)

(defparameter *cam* (send *pr2* :kinect_head-rgb))
(defparameter *image-buffer* (make-string (* (send *cam* :width) (send *cam* :height) 3)))
(defparameter *mask-buffer* (make-string (* (send *cam* :width) (send *cam* :height))))
(defparameter *marker-id* (ros::get-param "masked_object_marker_id" 0))
(ros::set-param "masked_object_marker_id" (inc *marker-id*))

(load-scene)

;; init viewer
(send *ri* :objects (list *scene*))

(defun create-mask-image (objs)
  (send *pr2* :reset-coords)
  (let* ((cds (send *ri* :state :worldcoords))
         (base-frame-id (pr2-interface-base-frame-id *ri*))
         (stamp (send *tfl* :get-latest-common-time "/world" base-frame-id))
         (mimg (instance image::grayscale-image :init (send *cam* :width) (send *cam* :height) *mask-buffer*))
         mask)
    (while (null cds)
           (setq cds (send *ri* :state :worldcoords)))
    (send *pr2* :transform cds :world)
    (send *pr2* :angle-vector (send *ri* :state :potentio-vector))
    (ros::ros-debug "[~A] robot pose: ~A, joints: ~A"
                    (ros::get-name) (send *pr2* :worldcoords) (send *ri* :state :potentio-vector))
    (send *cam* :draw-objects objs)
    (setq mask
          (send (send *cam* :image-viewer :viewsurface :getglimage
                      :imagebuf *image-buffer*
                      :offscreen t) :monochromize mimg))
    (if (> (send mask :average-pixel) 0.1)
        (ros::image->sensor_msgs/Image mask
                                       :seq (inc *mask-image-seq*)
                                       :stamp stamp
                                       :frame_id *mask-image-frame-id*)
      nil)))

(defun publish-mask-image ()
  (let* ((obj (send *scene* :object *object-name*))
         (msg (create-mask-image (list obj))))
    (when msg
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
