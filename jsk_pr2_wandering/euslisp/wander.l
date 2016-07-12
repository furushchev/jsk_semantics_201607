(require :pr2-interface "package://pr2eus/pr2-interface.l")
(require :pr2-move "package://jsk_demo_common/euslisp/pr2-move.l")
(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")


(defun look-around ()
  (send *pr2* :head :neck-p :joint-angle 0)
  (let* ((div 6)
         (step-angle (/ (- (send *pr2* :head :neck-y :max-angle)
                           (send *pr2* :head :neck-y :min-angle))
                        div)))
    (dotimes (i div)
      (send *pr2* :head :neck-y :joint-angle
            (+ (send *pr2* :head :neck-y :min-angle)
               (* i step-angle)))
      (send *ri* :angle-vector (send *pr2* :angle-vector) 1000)
      (send *ri* :wait-interpolation))))
#|
(defun shuffle (lst)
  (let ((src (copy-list lst))
        (ret (copy-list lst))
        obj)
    (while (setq obj (pop src))
      (setf (elt ret (random (length ret))) obj))
    ret))

(defun go-to-random-spot (&key (repeat 1) (wait 5000))
  (let* ((domain "/eng2/7f/room73B2")
         (spots (shuffle
                 (remove-if-not
                  #'(lambda (s)
                      (string= domain
                               (subseq (send s :name) 0 (length domain))))
                  (send *scene* :spots)))))
    (dolist (i repeat)
      (move-to-spot (pop spots))
      (unix:usleep (* wait 1000)))))
|#
(defun wander ()
  (let ((rand-x (random 1.0))
        (rand-y (random 1.0))
        (rand-th (random 360)))
    (send *ri* :go-pos rand-x rand-y rand-th)
    (send *ri* :wait-interpolation))
  (look-around))
    
(defun init ()
  (pr2-init)
  (setq *scene* (make-eng2-scene)))
