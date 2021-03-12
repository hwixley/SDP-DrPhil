//
//  RegistrationViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 11/03/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit

class RegistrationViewController: UIViewController, UITextFieldDelegate {
    
    //MARK: UI components
    @IBOutlet weak var verifyStack: UIStackView!
    @IBOutlet weak var createStack: UIStackView!
    @IBOutlet weak var idTextfield: UITextField!
    @IBOutlet weak var keyTextfield: UITextField!
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
        idTextfield.delegate = self
        keyTextfield.delegate = self
        passTextfield1.delegate = self
        passTextfield2.delegate = self
        self.tapOutsideKB.isEnabled = false
    }
    

    //MARK: Actions
    @IBAction func clickVerify(_ sender: Any) {
        if true {
            verifyStack.isHidden = true
            createStack.isHidden = false
        }
    }
    
    @IBAction func clickCreate(_ sender: Any) {
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
    
    
    //MARK: Private Methods
    func isValidPassword(_ pass: String) -> Bool {
        let passRegex = "^(.{0,8}|[^0-9]*|[^A-Z]*|[^a-z]*)$"
        return !NSPredicate(format: "SELF MATCHES %@", passRegex).evaluate(with: pass)
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
