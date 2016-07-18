;; test-publish-bounding-box.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(ros::roseus "test_pub_bbox")

(ros::load-ros-manifest "jsk_recognition_msgs")

(setq *topic* (format nil "~A/output" (ros::get-name)))

(ros::advertise *topic* jsk_recognition_msgs::BoundingBoxArray 1)

(setq *cube* (make-cube 1000 1000 1000))
(send *cube* :rotate (deg2rad 30) :z)
(send *cube* :translate #f(2000 -30 0) :world)

(setq *seq* 0)

(defun cubes->bboxes (cubes)
  (let ((header (instance std_msgs::header :init
                          :seq (inc *seq*)
                          :stamp (ros::time-now)
                          :frame_id "camera_link"))
        (msg (instance jsk_recognition_msgs::BoundingBoxArray :init)))
    (send msg :header header)
    (send msg :boxes
          (mapcar #'(lambda (c)
                      (instance jsk_recognition_msgs::BoundingBox :init
                                :header header
                                :pose (ros::coords->tf-pose (send c :worldcoords))
                                :dimensions (ros::pos->tf-translation
                                             (send (send c :box) :diagonal))))
                  cubes))
    msg))

(defun pub ()
  (ros::rate 10)
  (let (msg)
    (while (ros::ok)
     (setq msg (cubes->bboxes (list *cube*)))
     (ros::publish *topic* msg)
     (ros::sleep))))

(ros::ros-info "publishing box...")
(pub)
