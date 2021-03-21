//
//  ctrlReturnViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 01/02/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit
import Firebase

class ctrlReturnViewController: UIViewController, UITextFieldDelegate {

    //MARK: Properties
    @IBOutlet weak var headerLabel: UILabel!
    @IBOutlet weak var timeSegmentControl: UISegmentedControl!
    @IBOutlet weak var durationSegmentControl: UISegmentedControl!
    @IBOutlet weak var timeTextfield: UITextField!
    @IBOutlet weak var durationTextfield: UITextField!
    @IBOutlet var tapOutsideKB: UITapGestureRecognizer!
    @IBOutlet weak var timeSegmentLabel: UILabel!
    @IBOutlet weak var durationSegmentLabel: UILabel!
    @IBOutlet weak var timeLabel: UILabel!
    @IBOutlet weak var durationLabel: UILabel!
    @IBOutlet weak var timeStack: UIStackView!
    @IBOutlet weak var durationStack: UIStackView!
    @IBOutlet weak var timeTextStack: UIStackView!
    @IBOutlet weak var durationTextStack: UIStackView!
    
    
    
    //MARK: Pickers
    var dp = UIDatePicker()
    
    
    override func viewDidLoad() {
        super.viewDidLoad()

        timeTextfield.delegate = self
        durationTextfield.delegate = self
        self.timeTextfield.inputView = dp
        self.tapOutsideKB.isEnabled = false
        self.timeTextStack.isHidden = true
        self.durationTextStack.isHidden = true
        timeSegmentControl.setTitleTextAttributes([NSAttributedString.Key.foregroundColor: UIColor.white], for: .selected)
        timeSegmentControl.setTitleTextAttributes([NSAttributedString.Key.foregroundColor: UIColor.white], for: .normal)
        durationSegmentControl.setTitleTextAttributes([NSAttributedString.Key.foregroundColor: UIColor.white], for: .selected)
        durationSegmentControl.setTitleTextAttributes([NSAttributedString.Key.foregroundColor: UIColor.white], for: .normal)
        self.setupDPbounds()
    }
    
    //MARK: Actions
    @IBAction func tapSubmit(_ sender: UIBarButtonItem) {
        timeSegmentLabel.textColor = UIColor.white
        durationSegmentLabel.textColor = UIColor.white
        timeLabel.textColor = UIColor.white
        
        var dateAndTime = dateFormatter.string(from: Date()) + " " + timeTextfield.text!
        
        if headerLabel.isHidden == false {
            self.navigationItem.prompt = "Error: your robot does not have a shift today"
            return
        }
        
        if timeSegmentControl.selectedSegmentIndex == 0 {
            self.navigationItem.prompt = "Error: you must select a return time"
            timeSegmentLabel.textColor = UIColor.systemPink
            return
        } else if timeSegmentControl.selectedSegmentIndex == 2 && timeTextfield.text == "" {
            self.timeLabel.textColor = UIColor.systemPink
            self.navigationItem.prompt = "Error: you must enter a valid return time"
            return
        } else if timeSegmentControl.selectedSegmentIndex == 1 {
            dateAndTime = "ASAP"
        }
        
        var duration = durationTextfield.text ?? ""
        
        if durationSegmentControl.selectedSegmentIndex == 0 {
            self.navigationItem.prompt = "Error: you must select a duration"
            durationSegmentLabel.textColor = UIColor.systemPink
            return
        } else if durationSegmentControl.selectedSegmentIndex == 2 && durationTextfield.text == "" {
            self.durationLabel.textColor = UIColor.systemPink
            self.navigationItem.prompt = "Error: you must enter a valid duration"
            return
        } else if durationSegmentControl.selectedSegmentIndex == 1 {
            duration = "REST"
        }
        Firestore.firestore().collection("robots").document(MyUser.robot!.robotID).updateData(["returnTime": dateAndTime, "returnDuration": duration])
        
        MyUser.robot!.returnTime = dateAndTime
        MyUser.robot!.returnDuration = duration
        
        self.performSegue(withIdentifier: "submitStopSegue", sender: self)
    }
    
    @IBAction func tapOutsideKB(_ sender: UITapGestureRecognizer) {
        timeTextfield.resignFirstResponder()
        durationTextfield.resignFirstResponder()
        tapOutsideKB.isEnabled = false
    }
    
    @IBAction func segmentChanged(_ sender: UISegmentedControl) {
        if timeSegmentControl.selectedSegmentIndex == 2 {
            timeTextStack.isHidden = false
        } else {
            timeTextStack.isHidden = true
        }
    }
    
    @IBAction func durationSegmentChanged(_ sender: UISegmentedControl) {
        if durationSegmentControl.selectedSegmentIndex == 2 {
            durationTextStack.isHidden = false
        } else {
            durationTextStack.isHidden = true
        }
    }
    
    //MARK: Textfield
    func textFieldDidBeginEditing(_ textField: UITextField) {
        tapOutsideKB.isEnabled = true
    }
    
    func textFieldDidEndEditing(_ textField: UITextField) {
        if textField == timeTextfield {
            timeTextfield.text = dateFormatter3.string(from: dp.date)
        }
    }
    
    //MARK: Date picker
    func setupDPbounds() {
        dp.datePickerMode = UIDatePicker.Mode.time
        dp.locale = Locale(identifier: "en_GB")
        
        dp.minimumDate = Date()
        
        if isWeekday() {
            if MyUser.robot!.schedule!.weekdays == nil {
                timeSegmentControl.isEnabled = false
                headerLabel.text = "Your robot does not have a shift on weekdays"
            } else {
                dp.maximumDate = dateFormatter2.date(from: dateFormatter.string(from: Date()) + " " + MyUser.robot!.schedule!.weekdays!.end)
            }
        } else {
            if MyUser.robot!.schedule!.weekends == nil {
                timeStack.isHidden = true
                durationStack.isHidden = true
                headerLabel.isHidden = false
                headerLabel.text = "Your robot does not have a shift on weekends"
            } else {
                headerLabel.isHidden = true
                timeStack.isHidden = false
                durationStack.isHidden = false
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
