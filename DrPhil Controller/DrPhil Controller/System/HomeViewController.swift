//
//  HomeViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 27/01/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit

class HomeViewController: UIViewController {

    //MARK: Properties
    @IBOutlet weak var shiftLabel: UILabel!
    @IBOutlet weak var roundsLabel: UILabel!
    @IBOutlet weak var statusLabel: UILabel!
    @IBOutlet weak var mapImageView: UIImageView!
    @IBOutlet weak var shiftStack: UIStackView!
    @IBOutlet weak var roundsTextLabel: UILabel!
    @IBOutlet weak var batteryLabel: UILabel!
    @IBOutlet weak var disinfectantLabel: UILabel!
    @IBOutlet weak var statusStack: UIStackView!
    @IBOutlet weak var mapStack: UIStackView!
    
    
    override func viewDidLoad() {
        super.viewDidLoad()

        self.setupUI()
    }
    
    
    //MARK: Private methods
    func setupUI() {
        shiftStack.isHidden = false
        roundsTextLabel.text = "# Cleaning rounds:"
        
        statusLabel.text = UserInfo.status ?? "NA"
        
        if UserInfo.resources != nil {
            batteryLabel.text = String(UserInfo.resources!.battery) + "%"
            disinfectantLabel.text = String(UserInfo.resources!.disinfectant) + "%"
        }
        
        if UserInfo.map != nil {
            mapImageView.image = UserInfo.map!
        } else {
            mapStack.isHidden = true
        }
        
        if UserInfo.schedule != nil {
            if isWeekday() {
                if UserInfo.schedule!.weekdays != nil {
                    shiftLabel.text = UserInfo.schedule!.weekdays!.start + " - " + UserInfo.schedule!.weekdays!.end
                    roundsLabel.text = String(UserInfo.schedule!.weekdays!.numRounds)
                    return
                }
            } else {
                if UserInfo.schedule!.weekends != nil {
                    shiftLabel.text = UserInfo.schedule!.weekends!.start + " - " + UserInfo.schedule!.weekends!.end
                    roundsLabel.text = String(UserInfo.schedule!.weekends!.numRounds)
                    return
                }
            }
        }
        mapStack.isHidden = true
        shiftStack.isHidden = true
        statusStack.isHidden = true
        roundsTextLabel.text = "No shift today"
        roundsLabel.text = ""
        statusLabel.text = "idle at charging station"
        mapImageView.image = nil
    }
}
