;; hoge.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>


(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")

(pr2-init)

(defparameter *mask-image-topic* "/kinect_head/rgb/mask_fridge")
(defparameter *mask-image-frame-id* "/head_mount_kinect_rgb_optical_frame")
(defparameter *mask-image-seq* 0)

(defparameter *scene* (make-eng2-scene))
(defparameter *robot* (send *ri* :robot))
(defparameter *cam* (send *robot* :kinect_head-rgb))


;; init viewer
(send *ri* :objects (list *scene*))
(send *cam* :create-viewer)

;; assoc objects / spots
(setq *door-front-spot* (send *scene* :spot "/eng2/7f/room73B2-door-front"))
(setq *fridge* (send *scene* :object "room73b2-hitachi-fiesta-refrigerator"))

(if (send *ri* :simulation-modep)
    (progn
      (send *robot* :transform (send *door-front-spot* :worldcoords))
      (send *ri* :go-pos-unsafe 0 0 180))
    (send *ri* :update-robot-state))

(defun create-mask-image (objs)
  (send *cam* :draw-objects objs)
  (send (send (send *cam* :get-image) :monochromize)))

(defun publish-fridge-mask-image ()
  (let ((mask (create-mask-image (list *fridge*)))
        (msg (ros::image->sensor_msgs/Image mask
                                            :seq (inc *mask-image-seq*)
                                            :frame_id *mask-image-frame-id*)))
    (unless (ros::get-topic-publisher *mask-image-topic*)
      (ros::advertise *mask-image-topic* sensor_msgs::Image 5)
      (unix:sleep 1))
    (ros::publish *mask-image-topic* msg)
    t))


(provide :hoge) ;; end of hoge.l
