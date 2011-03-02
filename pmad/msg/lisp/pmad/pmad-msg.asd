
(in-package :asdf)

(defsystem "pmad-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils)
  :components ((:file "_package")
    (:file "Status" :depends-on ("_package"))
    (:file "_package_Status" :depends-on ("_package"))
    ))
