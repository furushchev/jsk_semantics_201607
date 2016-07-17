;; publish_object_bounding_box.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :scene "package://jsk_pr2_wandering/euslisp/scene")

(defparameter *object-name* (ros::get-param "~object_name"))
(defparameter *bounding-box-topic* (format nil "~A/~A" (ros::get-name) "output"))
(defparameter *publish-marker-p* (ros::get-param "~publish_marker" nil))
(defparameter *marker-topic* (format nil "~A/~A" (ros::get-name) "marker"))
(unless *object-name*
  (error "you must specify object name")
  (exit 1))

(unless (ros::get-topic-publisher *bounding-box-topic*)
  (ros::advertise *bounding-box-topic* jsk_recognition_msgs::BoundingBoxArray 1)
  (unix:sleep 1))
(when (and *publish-marker-p* (null (ros::get-topic-publisher *marker-topic*)))
  (defparameter *marker-id* (ros::get-param "masked_object_marker_id" 0))
  (ros::set-param "masked_object_marker_id" (inc *marker-id*))
  (ros::advertise *marker-topic* visualization_msgs::Marker 1)
  (unix:sleep 1))

(load-scene)

(defun object->bounding-cube (obj)
  (let ((cds (send obj :copy-worldcoords)))
    (ros::ros-info "obj cds: ~A" cds)
    (send obj :reset-coords)
    (ros::ros-info "obj world cds: ~A" (send obj :worldcoords))
    (let* ((boxes (send-all (send obj :bodies) :box))
           (minpoint (car (sort (send-all boxes :minpoint) #'v<)))
           (maxpoint (car (sort (send-all boxes :maxpoint) #'v>)))
           (centroid (scale 0.5 (v+ minpoint maxpoint)))
           (diagonal (v- maxpoint minpoint))
           (c (make-cube (aref diagonal 0) (aref diagonal 1) (aref diagonal 2) :pos centroid)))
      (ros::ros-info "min: ~A max: ~A cen: ~A" minpoint maxpoint centroid)
      (send obj :transform cds :world)
      (send c :transform cds :world)
      c)))

(defun object->ros-bounding-box (obj &key header (seq 0) (stamp (ros::time-now)) (frame_id "world"))
  (unless header (setq header (instance std_msgs::header :init
                                        :seq seq :stamp stamp :frame_id frame_id)))
  (let ((c (object->bounding-cube obj)))
    (ros::ros-info "cube: ~A cenetroid: ~A" c (send c :centroid))
    (instance jsk_recognition_msgs::BoundingBox :init
              :header header
              :pose (ros::coords->tf-pose (send c :worldcoords))
              :dimensions (ros::pos->tf-translation (send (send c :box) :diagonal)))))

(defun objects->ros-bounding-box-array (objs &key header (seq 0) (stamp (ros::time-now)) (frame_id "world"))
  (unless header (setq header (instance std_msgs::header :init
                                        :seq seq :stamp stamp :frame_id frame_id)))
  (instance jsk_recognition_msgs::BoundingBoxArray :init
            :header header
            :boxes (mapcar #'object->ros-bounding-box objs)))

(defun publish ()
  (ros::ros-info "[~A] publishing bounding box of ~A" (ros::get-name) *object-name*)
  (let ((bboxes (objects->ros-bounding-box-array (list (send *scene* :object *object-name*)))))
    (ros::rate 10)
    (while (ros::ok)
     (ros::publish *bounding-box-topic* bboxes)
     (ros::sleep))))

(provide :publish_object_bounding_box) ;; end of publish_object_bounding_box.l
