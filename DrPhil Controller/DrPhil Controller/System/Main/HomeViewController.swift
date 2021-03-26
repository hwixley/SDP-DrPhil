//
//  HomeViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 27/01/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit
import Firebase

class HomeViewController: UIViewController {

    //MARK: UI Components
    @IBOutlet weak var shiftLabel: UILabel!
    @IBOutlet weak var roundsLabel: UILabel!
    @IBOutlet weak var statusLabel: UILabel!
    @IBOutlet weak var shiftStack: UIStackView!
    @IBOutlet weak var shiftTextLabel: UILabel!
    @IBOutlet weak var roundsTextLabel: UILabel!
    @IBOutlet weak var batteryLabel: UILabel!
    @IBOutlet weak var disinfectantLabel: UILabel!
    @IBOutlet weak var statusStack: UIStackView!
    @IBOutlet weak var taskStack: UIStackView!
    @IBOutlet weak var taskStatusLabel: UILabel!
    @IBOutlet weak var shiftView: UIView!
    @IBOutlet weak var statusView: UIView!
    @IBOutlet weak var taskView: UIView!
    @IBOutlet weak var shiftTextStack: UIStackView!
    
    
    override func viewDidLoad() {
        super.viewDidLoad()

        self.setupUI()
    }
    
    override func viewDidAppear(_ animated: Bool) {
        self.setupUI()
    }
    
    
    //MARK: Private methods
    func setupUI() {
        shiftView.isHidden = false
        taskView.isHidden = false
        
        roundsTextLabel.text = "Interval between cleaning rounds (mins):"
        
        if MyUser.robot != nil {
            self.taskStatusLabel.isHidden = false
            self.loadStats()
            
            if MyUser.robot!.returnTime != "" && MyUser.robot!.returnDuration != "" {
                var status = ""
                if MyUser.robot!.returnTime == "ASAP" {
                    status = "Return to station ASAP"
                } else {
                    status = "Return to station at " + MyUser.robot!.returnTime!.suffix(5)
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
                    } else {
                        roundsTextLabel.text = "No shift today"
                        roundsLabel.text = ""
                        shiftTextStack.isHidden = true
                    }
                } else {
                    if MyUser.robot!.schedule!.weekends != nil {
                        shiftLabel.text = MyUser.robot!.schedule!.weekends!.start + " - " + MyUser.robot!.schedule!.weekends!.end
                        roundsLabel.text = String(MyUser.robot!.schedule!.weekends!.numRounds)
                    } else {
                        roundsTextLabel.text = "No shift today"
                        roundsLabel.text = ""
                        shiftTextStack.isHidden = true
                    }
                }
            } else {
                roundsTextLabel.text = "No shift today"
                roundsLabel.text = ""
                shiftTextStack.isHidden = true
            }
            
            if MyUser.statusInfo != nil {
                statusLabel.text = MyUser.statusInfo!.getStatus()
                
                if MyUser.statusInfo!.status != -1 && MyUser.statusInfo!.resources != nil {
                    batteryLabel.text = String(MyUser.statusInfo!.resources!.battery) + "%"
                    disinfectantLabel.text = String(MyUser.statusInfo!.resources!.disinfectant) + "%"
                }
            } else {
                statusLabel.text = "NA"
                batteryLabel.text = "NA"
                disinfectantLabel.text = "NA"
            }
        }
    }
    
    func loadStats() {
        let db = Firestore.firestore()
        
        db.collection("statuses").document(MyUser.robot!.robotID).addSnapshotListener { (docSnapshot, err) in
            
            if err != nil {
                
            } else if docSnapshot != nil && docSnapshot!.exists {
                let status = initStatusInfo(docData: docSnapshot!.data())
                
                self.statusLabel.text = status.getStatus()
                if status.resources != nil {
                    if status.resources!.battery != -1 {
                        self.batteryLabel.text = String(Int(status.resources!.battery)) + "%"
                    }
                    if status.resources!.disinfectant != -1 {
                        self.disinfectantLabel.text = String(Int(status.resources!.disinfectant)) + " ml"
                    }
                }
                MyUser.statusInfo = status
            }
        }
    }
}
