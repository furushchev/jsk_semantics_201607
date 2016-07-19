;; publish-room-object.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::load-ros-manifest "visualization_msgs")

(require :pr2-interface "package://pr2eus/pr2-interface.l")
(load "scene.l")

(ros::roseus "publish_room_object")

(defparameter *marker-array-topic* "/room_marker_array")
(defparameter *marker-array-seq* 0)

(load-scene)

;; fix pose
(setq *fridge* (send *scene* :object "room73b2-hitachi-fiesta-refrigerator"))
(send *fridge* :translate (float-vector 0 500 0))
(setq *table* (send *scene* :object "room73b2-karimoku-table"))
(send *table* :translate (float-vector -200 -200 0))

(defun pub ()
  (let (
        objs
        (id 0)
        (header (instance std_msgs::header :init))
        (msg (instance visualization_msgs::MarkerArray :init)))
    (push (send *scene* :object "room73b2-hitachi-fiesta-refrigerator") objs)
    (push (send *scene* :object "room73b2-karimoku-table") objs)
    (push (send *scene* :object "room73b2-door-left") objs)
    (push (send *scene* :object "room73b2-door-right") objs)
    (push (send *scene* :object "room73b2-hrp2-parts-drawer") objs)
    (send header :seq (inc *marker-array-seq*))
    (send header :stamp (ros::time-now))
    (send header :frame_id "world")
    (send msg :markers (mapcar #'(lambda (o)
                                   (object->marker-msg o header
                                                       :ns "73B2"
                                                       :lifetime 3000
                                                       :id (inc id)))
                               objs))
;;    (print-ros-msg msg)
    (unless (ros::get-topic-publisher *marker-array-topic*)
      (ros::advertise *marker-array-topic* visualization_msgs::MarkerArray 5)
      (unix:sleep 1))
    (ros::publish *marker-array-topic* msg)
    t))

(provide :publish-room-object) ;; end of publish-room-object.l
