//
//  StatsViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 27/01/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit

class StatsViewController: UIViewController {

    //MARK: Properties
    @IBOutlet weak var roundLabel: UILabel!
    @IBOutlet weak var roomLabel: UILabel!
    @IBOutlet weak var handlesLabel: UILabel!
    @IBOutlet weak var statsImageView: UIImageView!
    @IBOutlet weak var batteryUsageLabel: UILabel!
    @IBOutlet weak var disinfectantUsageLabel: UILabel!
    
    
    override func viewDidLoad() {
        super.viewDidLoad()

        self.setupUI()
    }


    //MARK: Private methods
    func setupUI() {
        if UserInfo.dayStats != nil {
            roundLabel.text = "Cleaning round: " + String(UserInfo.dayStats!.round) + "/" + String(UserInfo.dayStats!.maxRound)
            roomLabel.text = "Cleaning round completion status: " + String(UserInfo.dayStats!.room) + "/" + String(UserInfo.dayStats!.maxRooms)
            handlesLabel.text = "Total door handles disinfected: " + String(UserInfo.dayStats!.numHandles)
            batteryUsageLabel.text = "Battery usage: " + String(UserInfo.dayStats!.batteryUsage) + "W"
            disinfectantUsageLabel.text = "Disinfectant usage: " + String(UserInfo.dayStats!.disinfectantUsage) + "ml"
        }
    }
}
