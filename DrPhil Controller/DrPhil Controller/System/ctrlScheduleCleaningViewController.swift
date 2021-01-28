//
//  ctrlScheduleCleaningViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 27/01/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit

class ctrlScheduleCleaningViewController: UIViewController, UITextFieldDelegate {

    //MARK: Properties
    @IBOutlet weak var numRoundsTextField: UITextField!
    @IBOutlet weak var weekdaysStartTextField: UITextField!
    @IBOutlet weak var weekdaysEndTextField: UITextField!
    @IBOutlet weak var weekendsStartTextField: UITextField!
    @IBOutlet weak var weekendsEndTextField: UITextField!
    @IBOutlet weak var weekdaysSwitch: UISwitch!
    @IBOutlet weak var weekendsSwitch: UISwitch!
    @IBOutlet weak var weekdaysStack: UIStackView!
    @IBOutlet weak var weekendsStack: UIStackView!
    @IBOutlet var tapOutsideKB: UITapGestureRecognizer!
    
    //MARK: Temporary vars
    var clickedTxtf: UITextField?
    
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        numRoundsTextField.delegate = self
        weekdaysStartTextField.delegate = self
        weekdaysEndTextField.delegate = self
        weekendsEndTextField.delegate = self
        weekendsStartTextField.delegate = self
        weekdaysStack.isHidden = true
        weekendsStack.isHidden = true
        weekdaysSwitch.isOn = false
        weekendsSwitch.isOn = false
        self.tapOutsideKB.isEnabled = false
    }
    
    //MARK: Actions
    @IBAction func switchedWeekdays(_ sender: UISwitch) {
        if weekdaysSwitch.isOn {
            weekdaysStack.isHidden = false
        } else {
            weekdaysStack.isHidden = true
        }
    }
    
    @IBAction func switchedWeekends(_ sender: UISwitch) {
        if weekendsSwitch.isOn {
            weekendsStack.isHidden = false
        } else {
            weekendsStack.isHidden = true
        }
    }
    
    @IBAction func didTapOutsideKB(_ sender: UITapGestureRecognizer) {
        clickedTxtf!.resignFirstResponder()
        tapOutsideKB.isEnabled = false
    }
    
    //MARK: Textfield
    func textFieldDidBeginEditing(_ textField: UITextField) {
        self.clickedTxtf = textField
        tapOutsideKB.isEnabled = true
    }
    
    //MARK: FOcused keyboard
    func focusKB() {
        NotificationCenter.default.addObserver(self, selector: #selector(ctrlScheduleCleaningViewController.keyboardWillShow), name: UIResponder.keyboardWillShowNotification, object: nil)
        NotificationCenter.default.addObserver(self, selector: #selector(ctrlScheduleCleaningViewController.keyboardWillHide), name: UIResponder.keyboardWillHideNotification, object: nil)
    }
    
    @objc func keyboardWillShow(notification: NSNotification) {
        guard let keyboardSize = (notification.userInfo?[UIResponder.keyboardFrameEndUserInfoKey] as? NSValue)?.cgRectValue else {
            return
        }
        
        if self.view.frame.origin.y == 0 || self.clickedTxtf!.isEqual(weekendsEndTextField) {
            self.view.frame.origin.y -= keyboardSize.height
        }
    }
    
    @objc func keyboardWillHide(notification: NSNotification) {
        if self.view.frame.origin.y != 0 {
            self.view.frame.origin.y = 0
        }
    }
}
