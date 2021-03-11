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
        if MyUser.statusInfo != nil {
            if MyUser.statusInfo!.dayStats != nil {
                roundLabel.text = String(MyUser.statusInfo!.dayStats!.round) + "/" + String(MyUser.statusInfo!.dayStats!.maxRound)
                roomLabel.text = String(MyUser.statusInfo!.dayStats!.room) + "/" + String(MyUser.statusInfo!.dayStats!.maxRooms)
                handlesLabel.text = String(MyUser.statusInfo!.dayStats!.numHandles)
                batteryUsageLabel.text = String(MyUser.statusInfo!.dayStats!.batteryUsage) + "W"
                disinfectantUsageLabel.text = String(MyUser.statusInfo!.dayStats!.disinfectantUsage) + "ml"
            }
        }
    }
}
