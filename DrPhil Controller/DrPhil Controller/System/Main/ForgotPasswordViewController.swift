//
//  ForgotPasswordViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 24/03/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit
import FirebaseAuth

class ForgotPasswordViewController: UIViewController, UITextFieldDelegate {

    //MARK: UI Components
    @IBOutlet weak var emailTextfield: UITextField!
    @IBOutlet weak var emailLabel: UILabel!
    
    
    override func viewDidLoad() {
        super.viewDidLoad()

        emailTextfield.delegate = self
    }
    
    //MARK: Actions
     @IBAction func sendCodeButton(_ sender: UIBarButtonItem) {
         //Validate Input
         
         if isValidEmail(emailTextfield.text!) {
             
             Auth.auth().fetchSignInMethods(forEmail: emailTextfield.text!) { (providers, error) in
                 
                 if error != nil {
                     print(error!.localizedDescription)
                 } else if providers == nil {
                     self.navigationItem.prompt = "This email is not attached to any DrPhil account"
                     self.emailLabel.textColor = UIColor.systemPink
                     return
                 }
                 
                 //Perform segue to reset password VC
                 self.performSegue(withIdentifier: "resetPassSegue", sender: self)
                 return
             }
         } else {
             emailLabel.textColor = UIColor.systemPink
             self.navigationItem.prompt = "You have entered an invalid email"
         }
     }
     //MARK: Navigation
     override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
         if segue.identifier == "resetPassSegue" {
             let nc = segue.destination as! UINavigationController
             let vc = nc.viewControllers.first as! ResetPasswordViewController
             
             vc.email = emailTextfield.text!
         }
     }
     
     @IBAction func ForgotPassUnwindSegue(_ segue: UIStoryboardSegue) {
     }
     
     //MARK: Textfield
     func isValidEmail(_ email: String) -> Bool {
         let emailRegEx = "[A-Z0-9a-z._%+-]+@[A-Za-z0-9.-]+\\.[A-Za-z]{2,64}"
         let emailPred = NSPredicate(format: "SELF MATCHES %@", emailRegEx)
         return emailPred.evaluate(with: email)
     }
     
     func textFieldShouldReturn(_ textField: UITextField) -> Bool {
         textField.resignFirstResponder()
         return true
     }
     
     func textField(_ textField: UITextField, shouldChangeCharactersIn range: NSRange, replacementString string: String) -> Bool {
         let newText = (textField.text! as NSString).replacingCharacters(in: range, with: string)
         emailLabel.textColor = UIColor.white
         self.navigationItem.prompt = nil
         
         if !(newText.count < 80) {
             emailLabel.textColor = UIColor.systemPink
             self.navigationItem.prompt = "Warning: you are at the MAX allowed character count"
             return false
         }
         emailLabel.textColor = UIColor.white
         self.navigationItem.prompt = nil
         return true
     }
}
