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
    
    
    override func viewDidLoad() {
        super.viewDidLoad()

        self.setupUI()
    }
    

    /*
    // MARK: - Navigation

    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destination.
        // Pass the selected object to the new view controller.
    }
    */
    //MARK: Private methods
    func setupUI() {
        shiftStack.isHidden = false
        roundsTextLabel.text = "# Cleaning rounds:"
        
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
        shiftStack.isHidden = true
        roundsTextLabel.text = "No shift today"
        roundsLabel.text = ""
        statusLabel.text = "idle at charging station"
        mapImageView.image = nil
    }
}
