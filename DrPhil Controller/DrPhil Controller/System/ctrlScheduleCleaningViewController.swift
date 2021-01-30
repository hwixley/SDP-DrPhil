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
    
    //MARK: Pickers
    var wdStartPicker = UIDatePicker()
    var wdEndPicker = UIDatePicker()
    var weStartPicker = UIDatePicker()
    var weEndPicker = UIDatePicker()
    
    //MARK: Temporary vars
    var clickedTxtf: UITextField?
    
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        numRoundsTextField.delegate = self
        weekdaysStartTextField.delegate = self
        weekdaysEndTextField.delegate = self
        weekendsEndTextField.delegate = self
        weekendsStartTextField.delegate = self
        weekdaysStartTextField.inputView = wdStartPicker
        weekdaysEndTextField.inputView = wdEndPicker
        weekendsStartTextField.inputView = weStartPicker
        weekendsEndTextField.inputView = weEndPicker
        weekdaysStack.isHidden = true
        weekendsStack.isHidden = true
        weekdaysSwitch.isOn = false
        weekendsSwitch.isOn = false
        self.tapOutsideKB.isEnabled = false
        self.setupDPbounds()
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
    
    @IBAction func clickUpdate(_ sender: UIBarButtonItem) {
        if weekdaysSwitch.isOn {
            if weekdaysStartTextField.text == "" || weekdaysEndTextField.text == "" {
                self.navigationItem.prompt = "Fill in the highlighted fields or disable weekdays"
                return
            } else {
                UserInfo.schedule.weekdays = TimeFrame(start: weekdaysStartTextField.text!, end: weekdaysEndTextField.text!)
            }
        }
        
        if weekendsSwitch.isOn {
            if weekendsStartTextField.text == "" || weekendsEndTextField.text == "" {
                self.navigationItem.prompt = "Fill in the highlighted fields or disable weekends"
                return
            } else {
                UserInfo.schedule.weekends = TimeFrame(start: weekendsStartTextField.text!, end: weekendsEndTextField.text!)
            }
        }
        
        self.performSegue(withIdentifier: "updateScheduleSegue", sender: self)
    }
    
    
    //MARK: Textfield
    func textFieldDidBeginEditing(_ textField: UITextField) {
        self.clickedTxtf = textField
        tapOutsideKB.isEnabled = true
    }
    
    func textFieldDidEndEditing(_ textField: UITextField) {
        if textField === weekdaysStartTextField {
            weekdaysStartTextField.text = dateFormatter.string(from: wdStartPicker.date)
        } else if textField === weekdaysEndTextField {
            weekdaysEndTextField.text = dateFormatter.string(from: wdEndPicker.date)
        } else if textField === weekendsStartTextField {
            weekendsStartTextField.text = dateFormatter.string(from: weStartPicker.date)
        } else if textField === weekendsEndTextField {
            weekendsEndTextField.text = dateFormatter.string(from: weEndPicker.date)
        }
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
    
    //MARK: Date picker
    func setupDPbounds() {
        let dps = [wdStartPicker, wdEndPicker, weStartPicker, weEndPicker]
        
        for dp in dps {
            dp.datePickerMode = UIDatePicker.Mode.time
            dp.locale = Locale(identifier: "en_GB")
        }
    }
    
    lazy var dateFormatter: DateFormatter = {
        let df = DateFormatter()
        df.timeStyle = .short
        df.locale = Locale(identifier: "en_GB")
        return df
    }()
}
