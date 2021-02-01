//
//  ControlViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 27/01/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit

class ControlViewController: UIViewController {

    override func viewDidLoad() {
        super.viewDidLoad()

        
        UserInfo.schedule = CleanSchedule(weekdays: TimeFrame(start: "7:00", end: "22:00", numRounds: 30), weekends: nil)
        UserInfo.map = UIImage(named: "mapView")
        UserInfo.resources = ResourceMetrics(battery: 89, disinfectant: 63)
        UserInfo.status = "Cleaning room 3 on floor 2"
        UserInfo.dayStats = DailyStats(round: 2, maxRound: 6, room: 3, maxRooms: 10, numHandles: 24, batteryUsage: 100, disinfectantUsage: 122)
        // Do any additional setup after loading the view.
    }
    

    /*
    // MARK: - Navigation

    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destination.
        // Pass the selected object to the new view controller.
    }
    */
    @IBAction func unwindToControlSegue(_ segue: UIStoryboardSegue) {
    }

}
