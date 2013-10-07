
(cl:in-package :asdf)

(defsystem "CodyLegoBot-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "commandsend" :depends-on ("_package_commandsend"))
    (:file "_package_commandsend" :depends-on ("_package"))
  ))