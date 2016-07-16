;; scene.l
;; Author: Yuki Furuta <furushchev@jsk.imi.i.u-tokyo.ac.jp>

(require :eng2-scene "package://jsk_maps/src/eng2-scene.l")

(defun load-scene ()
  (defparameter *scene* (make-eng2-scene))
  ;; assoc objects / spots
  (setq *door-front-spot* (send *scene* :spot "/eng2/7f/room73B2-door-front"))
  (setq *fridge* (send *scene* :object "room73b2-hitachi-fiesta-refrigerator"))
  (send *fridge* :translate (float-vector 0 500 0))
  (setq *table* (send *scene* :object "room73b2-karimoku-table"))
  (send *table* :translate (float-vector -400 -150 0))
  (setq *drawer* (send *scene* :object "room73b2-hrp2-parts-drawer"))
  (send *drawer* :translate (float-vector 0 200 0)))

(provide :scene) ;; end of scene.l
