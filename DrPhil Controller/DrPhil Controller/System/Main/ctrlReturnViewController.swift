//
//  ctrlReturnViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 01/02/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit

class ctrlReturnViewController: UIViewController, UITextFieldDelegate {

    //MARK: Properties
    @IBOutlet weak var segmentControl: UISegmentedControl!
    @IBOutlet weak var scheduledStack: UIStackView!
    @IBOutlet weak var returnTextfield: UITextField!
    @IBOutlet var tapOutsideKB: UITapGestureRecognizer!
    @IBOutlet weak var segmentLabel: UILabel!
    @IBOutlet weak var returnLabel: UILabel!
    
    //MARK: Pickers
    var dp = UIDatePicker()
    
    
    override func viewDidLoad() {
        super.viewDidLoad()

        returnTextfield.delegate = self
        self.returnTextfield.inputView = dp
        self.tapOutsideKB.isEnabled = false
        self.scheduledStack.isHidden = true
        segmentControl.setTitleTextAttributes([NSAttributedString.Key.foregroundColor: UIColor.white], for: .selected)
        segmentControl.setTitleTextAttributes([NSAttributedString.Key.foregroundColor: UIColor.white], for: .normal)
        self.setupDPbounds()
    }
    
    //MARK: Actions
    @IBAction func tapSubmit(_ sender: UIBarButtonItem) {
        segmentLabel.textColor = UIColor.white
        returnLabel.textColor = UIColor.white
        
        var dateAndTime = returnTextfield!.text
        
        if segmentControl.selectedSegmentIndex == 0 {
            self.navigationItem.prompt = "Error: you must select a return time"
            segmentLabel.textColor = UIColor.systemPink
            return
        } else if segmentControl.selectedSegmentIndex == 2 && returnTextfield.text == "" {
            self.returnLabel.textColor = UIColor.systemPink
            self.navigationItem.prompt = "Error: you must enter a valid return time"
            return
        } else {
            dateAndTime = "ASAP"
        }
        
        
        self.performSegue(withIdentifier: "submitStopSegue", sender: self)
    }
    
    @IBAction func tapOutsideKB(_ sender: UITapGestureRecognizer) {
        returnTextfield.resignFirstResponder()
        tapOutsideKB.isEnabled = false
    }
    
    @IBAction func segmentChanged(_ sender: UISegmentedControl) {
        if segmentControl.selectedSegmentIndex == 2 {
            scheduledStack.isHidden = false
        } else {
            scheduledStack.isHidden = true
        }
    }
    
    //MARK: Textfield
    func textFieldDidBeginEditing(_ textField: UITextField) {
        tapOutsideKB.isEnabled = true
    }
    
    func textFieldDidEndEditing(_ textField: UITextField) {
        returnTextfield.text = dateFormatter3.string(from: dp.date)
    }
    
    //MARK: Date picker
    func setupDPbounds() {
        dp.datePickerMode = UIDatePicker.Mode.time
        dp.locale = Locale(identifier: "en_GB")
        
        dp.minimumDate = Date()
        
        if isWeekday() {
            if MyUser.robot!.schedule!.weekdays == nil {
                segmentControl.isEnabled = false
                segmentLabel.text = "Your robot does not have a shift on weekdays"
            } else {
                dp.maximumDate = dateFormatter2.date(from: dateFormatter.string(from: Date()) + " " + MyUser.robot!.schedule!.weekdays!.end)
            }
        } else {
            if MyUser.robot!.schedule!.weekends == nil {
                segmentControl.isEnabled = false
                segmentLabel.text = "Your robot does not have a shift on weekends"
            } else {
                dp.maximumDate = dateFormatter2.date(from: dateFormatter.string(from: Date()) + " " + MyUser.robot!.schedule!.weekends!.end)
            }
        }
        dp.minuteInterval = 5
    }
    
    lazy var dateFormatter: DateFormatter = {
        let df = DateFormatter()
        df.dateFormat = "YYYY-MM-dd"
        df.locale = Locale(identifier: "en_GB")
        return df
    }()
    
    lazy var dateFormatter2: DateFormatter = {
        let df = DateFormatter()
        df.dateFormat = "YYYY-MM-dd HH:mm"
        df.locale = Locale(identifier: "en_GB")
        return df
    }()
    
    lazy var dateFormatter3: DateFormatter = {
        let df = DateFormatter()
        df.dateFormat = "HH:mm"
        df.locale = Locale(identifier: "en_GB")
        return df
    }()
}
