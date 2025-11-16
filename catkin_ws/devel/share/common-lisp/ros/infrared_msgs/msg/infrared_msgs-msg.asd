
(cl:in-package :asdf)

(defsystem "infrared_msgs-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils :geometry_msgs-msg
               :std_msgs-msg
)
  :components ((:file "_package")
    (:file "IrDetection" :depends-on ("_package_IrDetection"))
    (:file "_package_IrDetection" :depends-on ("_package"))
    (:file "IrSignal" :depends-on ("_package_IrSignal"))
    (:file "_package_IrSignal" :depends-on ("_package"))
  ))