//
//  ResetPasswordViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 24/03/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit
import FirebaseAuth

class ResetPasswordViewController: UIViewController, UITextFieldDelegate {
    
    //MARK: Properties
    var email = String()
    
    //MARK: UI components
    @IBOutlet weak var codeTitleLabel: UILabel!

    override func viewDidLoad() {
        super.viewDidLoad()

        self.sendLink()
    }
    
   //MARK: Actions
   @IBAction func sendLinkButton(_ sender: UIButton) {
       //sender.preventRepeatedPresses()
       self.sendLink()
   }
   
   //MARK: Private methods
   func sendLink() {

       Auth.auth().sendPasswordReset(withEmail: email) { (error) in
           if error != nil {
               self.codeTitleLabel.text = "ERROR: For some reason we were unable to send a password reset link to your email: \(self.email)\n\nPlease check your internet connection and try again by clicking 'Send another link'."
           } else {
               UserDefaults.standard.set(self.email, forKey: "Email")
               self.codeTitleLabel.text = "A password reset link has been sent to your email: \(self.email)\n\nYou must open it to reset your password!"
           }
       }
   }
}
