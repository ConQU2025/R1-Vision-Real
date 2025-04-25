
(cl:in-package :asdf)

(defsystem "conqu-msg"
  :depends-on (:roslisp-msg-protocol :roslisp-utils )
  :components ((:file "_package")
    (:file "ByteArray" :depends-on ("_package_ByteArray"))
    (:file "_package_ByteArray" :depends-on ("_package"))
  ))