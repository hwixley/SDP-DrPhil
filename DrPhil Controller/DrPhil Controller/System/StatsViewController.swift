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

        // Do any additional setup after loading the view.
    }


    //MARK: Private methods
    func setupUI() {
        if UserInfo.dayStats != nil {
            if UserInfo.dayStats!.round != nil {
                
            }
        }
    }
}
