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
    //Gestures
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        self.idTextfield.delegate = self
        self.passTextfield.delegate = self
    }

    @IBAction func clickLogin(_ sender: UIBarButtonItem) {
        if idTextfield.text != "" && passTextfield.text != "" {
            Auth.auth().signIn(withEmail: idTextfield.text + "@dr.phil", password: passTextfield.text) { (authResult, err)}
            
            self.performSegue(withIdentifier: "login", sender: self)
        } else {
            self.navigationItem.prompt = "You must enter an ID and password"
        }
        
    }
    
}

