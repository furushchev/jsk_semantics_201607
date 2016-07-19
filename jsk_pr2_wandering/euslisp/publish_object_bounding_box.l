;; publish_object_bounding_box.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :scene "package://jsk_pr2_wandering/euslisp/scene")

(defparameter *object-names* (ros::get-param "~object_names"))
(defparameter *bounding-box-topic* (format nil "~A/~A" (ros::get-name) "output"))
(defparameter *publish-marker-p* (ros::get-param "~publish_marker" nil))
(defparameter *marker-topic* (format nil "~A/~A" (ros::get-name) "marker"))
(defparameter *padding-factor* (ros::get-param "~padding_factor" 1.0))
(defparameter *bounding-box-msg-seq* 0)
(defparameter *marker-msg-seq* 0)
(unless *object-names*
  (error "you must specify object_names parameter")
  (exit 1))
(unless (every #'stringp *object-names*)
  (error "parameter ~object_names must be array of string")
  (exit 1))

(unless (ros::get-topic-publisher *bounding-box-topic*)
  (ros::advertise *bounding-box-topic* jsk_recognition_msgs::BoundingBoxArray 1)
  (unix:sleep 1))
(when (and *publish-marker-p* (null (ros::get-topic-publisher *marker-topic*)))
  (ros::advertise *marker-topic* visualization_msgs::MarkerArray 1)
  (unix:sleep 1))

(load-scene)

(defun object->bounding-cube (obj)
  (let ((cds (send obj :copy-worldcoords))
        (parent (send obj :parent)))
    (send parent :dissoc obj)
    (ros::ros-debug "obj cds: ~A" cds)
    (send obj :newcoords (make-coords))
    (ros::ros-debug "obj world cds: ~A" (send obj :worldcoords))
    (ros::ros-debug "bodies world cds: ~A" (send-all (send obj :bodies) :worldcoords)) ;; DO NOT REMOVE THIS LINE
    (let* ((boxes (send-all (send obj :bodies) :box))
           (minpoint (car (sort (send-all boxes :minpoint) #'v<)))
           (maxpoint (car (sort (send-all boxes :maxpoint) #'v>)))
           (centroid (scale 0.5 (v+ minpoint maxpoint)))
           (diagonal (v- maxpoint minpoint))
           (c (make-cube (aref diagonal 0) (aref diagonal 1) (aref diagonal 2) :pos centroid)))
      (ros::ros-debug "min: ~A max: ~A cen: ~A" minpoint maxpoint centroid)
      (send obj :newcoords cds)
      (send c :transform cds)
      (send parent :assoc obj)
      (ros::ros-debug "c worldcoords: ~A" (send c :worldcoords))
      c)))

(defun object->ros-bounding-box (obj &key header (seq 0) (stamp (ros::time-now)) (frame_id "world"))
  (unless header (setq header (instance std_msgs::header :init
                                        :seq seq :stamp stamp :frame_id frame_id)))
  (let ((c (object->bounding-cube obj)))
    (ros::ros-debug "cube: ~A cenetroid: ~A" c (send c :centroid))
    (instance jsk_recognition_msgs::BoundingBox :init
              :header header
              :pose (ros::coords->tf-pose (send c :worldcoords))
              :dimensions (ros::pos->tf-translation (scale *padding-factor* (send (send c :box) :diagonal))))))

(defun objects->ros-bounding-box-array (objs &key header (seq (inc *bounding-box-msg-seq*)) (stamp (ros::time-now)) (frame_id "world"))
  (unless header (setq header (instance std_msgs::header :init
                                        :seq seq :stamp stamp :frame_id frame_id)))
  (instance jsk_recognition_msgs::BoundingBoxArray :init
            :header header
            :boxes (mapcar #'(lambda (obj)
                               (object->ros-bounding-box obj :header header))
                           objs)))

(defun objects->marker-array (objs &key header (seq (inc *marker-msg-seq*)) (stamp (ros::time-now)) (frame_id "world"))
  (unless header (setq header (instance std_msgs::header :init
                                        :seq seq :stamp stamp :frame_id frame_id)))
  (let ((id 0))
    (instance visualization_msgs::MarkerArray :init
              :markers (mapcar #'(lambda (obj)
                                   (object->marker-msg obj header :id (inc id) :lifetime 300 :ns "masked_objects"))
                               objs))))

(defun update-bounding-box-array-stamp (msg)
  (let ((tm (ros::time-now)))
    (send msg :header :stamp tm)
    (dolist (box (send msg :boxes))
      (send box :header :stamp tm))
    msg))

(defun update-marker-array-stamp (msg)
  (let ((tm (ros::time-now)))
    (dolist (m (send msg :markers))
      (send m :header :stamp tm))
    msg))

(defun publish ()
  (ros::ros-info "[~A] publishing ~A bounding boxes with padding factor ~A:" (ros::get-name) (length *object-names*) *padding-factor*)
  (dolist (n *object-names*) (ros::ros-info "~t- ~A" n))

  (let* ((objs (mapcar #'(lambda (n) (send *scene* :object n)) *object-names*))
         (bboxes (objects->ros-bounding-box-array objs))
         (markers (objects->marker-array objs)))
    (ros::rate 10)
    (while (ros::ok)
      (cond
       ((not (eq (ros::get-param-cached "~padding_factor" 1.0) *padding-factor*))
        (setq *padding-factor* (ros::get-param-cached "~padding_factor" 1.0))
        (ros::ros-info "updated padding_factor to ~A" *padding-factor*)
        (setq bboxes (objects->ros-bounding-box-array objs)))
       (t
        (setq bboxes (update-bounding-box-array-stamp bboxes))))
     (ros::publish *bounding-box-topic* bboxes)
     (when *publish-marker-p*
       (setq markers (update-marker-array-stamp markers))
       (ros::publish *marker-topic* markers))
     (ros::sleep))))

(provide :publish_object_bounding_box) ;; end of publish_object_bounding_box.l
