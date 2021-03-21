//
//  ViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 25/01/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit
import FirebaseAuth
import Firebase

class LoginViewController: UIViewController, UITextFieldDelegate {
    
    //MARK: UI Components
    @IBOutlet weak var idTextfield: UITextField!
    @IBOutlet weak var passTextfield: UITextField!
    @IBOutlet var tapOutsideKB: UITapGestureRecognizer!
    //MARK: Properties
    var clickedTxtf : UITextField? = nil
    
    var logged = false {
        didSet {
            if self.logged {
                self.performSegue(withIdentifier: "login", sender: self)
            }
        }
    }
    
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        self.idTextfield.delegate = self
        self.passTextfield.delegate = self
        self.tapOutsideKB.isEnabled = false
    }

    @IBAction func clickLogin(_ sender: UIBarButtonItem) {
        if idTextfield.text != nil && passTextfield.text != nil && idTextfield.text != "" && passTextfield.text != "" {
            
            FirebaseAuth.Auth.auth().signIn(withEmail: idTextfield.text!, password: passTextfield.text!) { (authResult, err) in
                
                if err != nil {
                    self.navigationItem.prompt = "Incorrect email or password"
                    print(err!.localizedDescription)
                } else if authResult != nil {
                    let db = Firestore.firestore()
                    
                    let userDoc = db.collection("users").document(authResult!.user.uid)
                    
                    userDoc.getDocument { (docSnapshot, err) in
                        
                        if err != nil {
                            print(err!.localizedDescription)
                        } else if docSnapshot != nil && docSnapshot!.exists {
                            let rid = docSnapshot!.data()!["rid"] as! String
                            
                            let robotDoc = db.collection("robots").document(rid)
                            
                            robotDoc.getDocument { (docSnapshot, err) in
                                
                                if err != nil {
                                    
                                } else if docSnapshot != nil && docSnapshot!.exists {
                                    let doc = docSnapshot!.data()
                                    
                                    MyUser.robot = initRobot(docData: doc)
                                    self.logged = true
                                }
                            }
                        }
                    }
                }
            }
        } else {
            self.navigationItem.prompt = "You must enter an ID and password"
        }
        
    }
    
    @IBAction func tapOutsideKB(_ sender: UITapGestureRecognizer) {
        if self.clickedTxtf != nil {
            self.clickedTxtf!.resignFirstResponder()
            self.tapOutsideKB.isEnabled = false
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


