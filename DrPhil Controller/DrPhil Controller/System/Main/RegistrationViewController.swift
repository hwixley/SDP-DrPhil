//
//  RegistrationViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 11/03/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit
import FirebaseAuth
import Firebase

class RegistrationViewController: UIViewController, UITextFieldDelegate {
    
    //MARK: UI components
    @IBOutlet weak var verifyStack: UIStackView!
    @IBOutlet weak var createStack: UIStackView!
    @IBOutlet weak var idTextfield: UITextField!
    @IBOutlet weak var keyTextfield: UITextField!
    @IBOutlet weak var emailTextfield: UITextField!
    @IBOutlet weak var emailOrb: UILabel!
    @IBOutlet weak var passTextfield1: UITextField!
    @IBOutlet weak var passTextfield2: UITextField!
    @IBOutlet weak var orb1: UILabel!
    @IBOutlet weak var orb2: UILabel!
    @IBOutlet var tapOutsideKB: UITapGestureRecognizer!
    
    //MARK: Properties
    var clickedTxtf : UITextField? = nil
    

    override func viewDidLoad() {
        super.viewDidLoad()

        createStack.isHidden = true
        verifyStack.isHidden = false
        idTextfield.delegate = self
        keyTextfield.delegate = self
        emailTextfield.delegate = self
        passTextfield1.delegate = self
        passTextfield2.delegate = self
        self.tapOutsideKB.isEnabled = false
    }
    
    //MARK: Navigation
    @IBAction func unwindToRegistration(_ segue: UIStoryboardSegue) {
    }
    

    //MARK: Actions
    @IBAction func clickVerify(_ sender: Any) {
        let db = Firestore.firestore()
        
        if idTextfield.text! != "" && keyTextfield.text! != "" {
            db.collection("unregistered-models").document(idTextfield.text!).getDocument { (docSnapshot, err) in
                
                if err != nil {
                    print(err!.localizedDescription)
                    self.navigationItem.prompt = "Error verifying credentials"
                } else if docSnapshot != nil && docSnapshot!.exists {
                    let key = docSnapshot!.data()!["key"] as! String
                    
                    if key == self.keyTextfield.text! {
                        self.verifyStack.isHidden = true
                        self.createStack.isHidden = false
                        self.clickedTxtf!.resignFirstResponder()
                        self.tapOutsideKB.isEnabled = false
                        self.navigationItem.prompt = nil
                    }
                } else {
                    self.navigationItem.prompt = "ERROR: There is no unregistered robot with this ID and key"
                }
            }
        } else {
            self.navigationItem.prompt = "ERROR: You must fill in the ID and Key fields"
        }
    }
    
    @IBAction func clickCreate(_ sender: Any) {
        if orb1.textColor == UIColor.green && orb2.textColor == UIColor.green && emailOrb.textColor == UIColor.green {
            Auth.auth().createUser(withEmail: emailTextfield.text!, password: passTextfield1.text!) { (authResult, err) in
                
                if err != nil {
                    self.navigationItem.prompt = "ERROR: This robot already has an account"
                    print(err!.localizedDescription)
                } else {
                    let db = Firestore.firestore()
                    
                    db.collection("robots").document(self.idTextfield.text!).setData(["uid": authResult!.user.uid, "rid": self.idTextfield.text!,"weekends": [], "weekdays": [], "returnTime": "", "returnDuration": ""])
                    
                    db.collection("users").document(authResult!.user.uid).setData(["rid": self.idTextfield.text!])
                    
                    db.collection("unregistered-models").document(self.idTextfield.text!).delete()
                    
                    MyUser.robot = Robot(UID: authResult!.user.uid, robotID: self.idTextfield.text!)
                    self.performSegue(withIdentifier: "registrationSuccess", sender: self)
                }
            }
        } else {
            self.navigationItem.prompt = "Please enter a valid email and password"
        }
    }
    
    @IBAction func tapOutsideKB(_ sender: UITapGestureRecognizer) {
        if self.clickedTxtf != nil {
            self.clickedTxtf!.resignFirstResponder()
            self.tapOutsideKB.isEnabled = false
        }
    }
    
    
    //MARK: Textfield
    @IBAction func editPass1(_ sender: UITextField) {
        passOrbs(type: true)
    }
    
    @IBAction func editPass2(_ sender: UITextField) {
        passOrbs(type: false)
    }
    
    @IBAction func editEmail(_ sender: UITextField) {
        emailOrbs()
    }
    
    //MARK: Private Methods
    func isValidPassword(_ pass: String) -> Bool {
        let passRegex = "^(.{0,8}|[^0-9]*|[^A-Z]*|[^a-z]*)$"
        return !NSPredicate(format: "SELF MATCHES %@", passRegex).evaluate(with: pass)
    }
    
    func isValidEmail(_ email: String) -> Bool {
        let emailRegex = "[A-Z0-9a-z._%+-]+@[A-Za-z0-9.-]+\\.[A-Za-z]{2,64}"
        return NSPredicate(format: "SELF MATCHES %@", emailRegex).evaluate(with: email)
    }
    
    func passOrbs(type: Bool) {
        var orbLabel = UILabel()
        var orb2Label = UILabel()
        var txtField1 = UITextField()
        var txtField2 = UITextField()
        var newText = String()
        
        if type {
            orbLabel = orb1
            orb2Label = orb2
            txtField1 = passTextfield1
            txtField2 = passTextfield2
        } else {
            orbLabel = orb2
            orb2Label = orb1
            txtField1 = passTextfield2
            txtField2 = passTextfield1
        }
        newText = txtField1.text!
        
        if !isValidPassword(newText) {
            orbLabel.textColor = UIColor.systemPink
        } else if (txtField1.text != txtField2.text) && (txtField2.text == "") {
            orbLabel.textColor = UIColor.green
            orb2Label.textColor = UIColor.white
        } else if (txtField1.text! != txtField2.text!) && (txtField2.text != "") {
            orbLabel.textColor = UIColor.orange
        } else if passTextfield2.text! == passTextfield1.text! {
            orbLabel.textColor = UIColor.green
            orb2Label.textColor = UIColor.green
        }
    }
    
    func emailOrbs() {
        if isValidEmail(emailTextfield.text!) {
            emailOrb.textColor = UIColor.green
        } else {
            emailOrb.textColor = UIColor.systemPink
        }
    }
    
    //MARK: Textfield
    func textFieldDidBeginEditing(_ textField: UITextField) {
        self.tapOutsideKB.isEnabled = true
        self.clickedTxtf = textField
    }
    
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        textField.resignFirstResponder()
        self.tapOutsideKB.isEnabled = false
        return true
    }
}
