//
//  MyFunctions.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 11/03/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import Foundation
import Firebase

func initRobot(docData: [String : Any]?) -> Robot? {
    let wd = docData!["weekdays"] as! [String]
    let we = docData!["weekends"] as! [String]
    var wdTF: TimeFrame? = nil
    var weTF: TimeFrame? = nil
    
    if !wd.isEmpty {
        wdTF = TimeFrame(start: wd[0], end: wd[1], numRounds: Int(wd[2])!)
    }
    if !we.isEmpty {
        weTF = TimeFrame(start: we[0], end: we[1], numRounds: Int(we[2])!)
    }
    let cleanSched = CleanSchedule(weekdays: wdTF, weekends: weTF)
    
    return Robot(UID: docData!["uid"] as! String, robotID: docData!["rid"] as! String, schedule: cleanSched)
}

func initStatusInfo(docData: [String : Any]?) -> StatusInfo {
    var dayStats: DailyStats? = nil
    var resources: ResourceMetrics? = nil
    
    
    
    return StatusInfo(status: docData!["status"] as! Int, dayStats: nil, resources: nil, map: nil)
}

func initTaskStack(docData: QueryDocumentSnapshot) -> [Task] {
    return []
}
