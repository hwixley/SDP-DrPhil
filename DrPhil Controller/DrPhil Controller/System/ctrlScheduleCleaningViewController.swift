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
    @IBOutlet weak var weekdaysSwitch: UISwitch!
    @IBOutlet weak var weekendsSwitch: UISwitch!
    @IBOutlet weak var weekdaysStack: UIStackView!
    @IBOutlet weak var weekendsStack: UIStackView!
    @IBOutlet var tapOutsideKB: UITapGestureRecognizer!
    
    
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        weekdaysStack.isHidden = true
        weekendsStack.isHidden = true
        weekdaysSwitch.isOn = false
        weekendsSwitch.isOn = false
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
    }
    
    //MARK: Textfield
    func textFieldDidBeginEditing(_ textField: UITextField) {
        tapOutsideKB.isEnabled = true
    }
}
