//
//  HomeViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 27/01/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit

class HomeViewController: UIViewController {

    //MARK: UI Components
    @IBOutlet weak var shiftLabel: UILabel!
    @IBOutlet weak var roundsLabel: UILabel!
    @IBOutlet weak var statusLabel: UILabel!
    @IBOutlet weak var shiftStack: UIStackView!
    @IBOutlet weak var roundsTextLabel: UILabel!
    @IBOutlet weak var batteryLabel: UILabel!
    @IBOutlet weak var disinfectantLabel: UILabel!
    @IBOutlet weak var statusStack: UIStackView!
    @IBOutlet weak var taskStack: UIStackView!
    @IBOutlet weak var tableView: UITableView!
    @IBOutlet weak var taskStatusLabel: UILabel!
    
    
    override func viewDidLoad() {
        super.viewDidLoad()

        self.setupUI()
    }
    
    override func viewDidAppear(_ animated: Bool) {
        self.setupUI()
    }
    
    
    //MARK: Private methods
    func setupUI() {
        shiftStack.isHidden = false
        taskStack.isHidden = false
        
        roundsTextLabel.text = "Interval between cleaning rounds (mins):"
        
        if MyUser.robot != nil {
            self.taskStatusLabel.isHidden = false
            self.tableView.isHidden = true
            
            if MyUser.robot!.returnTime != "" && MyUser.robot!.returnDuration != "" {
                var status = ""
                if MyUser.robot!.returnTime == "ASAP" {
                    status = "Return to station ASAP"
                } else {
                    status = "Return to station at " + MyUser.robot!.returnTime!
                }
                if MyUser.robot!.returnDuration == "REST" {
                    status += " for the rest of today's shift"
                } else {
                    status += " for " + MyUser.robot!.returnDuration! + " minutes"
                }
                self.taskStatusLabel.text = status
                
            } else {
                self.taskStatusLabel.text = "There are no queued tasks"
            }
            
            
            if MyUser.robot!.schedule != nil {
                if isWeekday() {
                    if MyUser.robot!.schedule!.weekdays != nil {
                        shiftLabel.text = MyUser.robot!.schedule!.weekdays!.start + " - " +  MyUser.robot!.schedule!.weekdays!.end
                        roundsLabel.text = String(MyUser.robot!.schedule!.weekdays!.numRounds)
                        return
                    }
                } else {
                    if MyUser.robot!.schedule!.weekends != nil {
                        shiftLabel.text = MyUser.robot!.schedule!.weekends!.start + " - " + MyUser.robot!.schedule!.weekends!.end
                        roundsLabel.text = String(MyUser.robot!.schedule!.weekends!.numRounds)
                        return
                    }
                }
            }
            
            if MyUser.statusInfo != nil {
                statusLabel.text = MyUser.statusInfo!.getStatus()
                
                if MyUser.statusInfo!.resources != nil {
                    batteryLabel.text = String(MyUser.statusInfo!.resources!.battery) + "%"
                    disinfectantLabel.text = String(MyUser.statusInfo!.resources!.disinfectant) + "%"
                }
            }
        }
        shiftStack.isHidden = true
        statusStack.isHidden = true
        taskStack.isHidden = true
        roundsTextLabel.text = "No shift today"
        roundsLabel.text = ""
        statusLabel.text = "idle at charging station"
    }
}
