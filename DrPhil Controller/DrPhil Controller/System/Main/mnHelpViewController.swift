//
//  mnHelpViewController.swift
//  DrPhil Controller
//
//  Created by GetSkooled on 26/03/2021.
//  Copyright © 2021 Klean. All rights reserved.
//

import UIKit

class mnHelpViewController: UIViewController {

    override func viewDidLoad() {
        super.viewDidLoad()

        // Do any additional setup after loading the view.
    }
    
    @IBAction func openWebsite(_ sender: UIButton) {
        if let url = URL(string: "https://group13.sdp.inf.ed.ac.uk/index.html") {
            UIApplication.shared.open(url)
        }
    }
    
    /*
    // MARK: - Navigation

    // In a storyboard-based application, you will often want to do a little preparation before navigation
    override func prepare(for segue: UIStoryboardSegue, sender: Any?) {
        // Get the new view controller using segue.destination.
        // Pass the selected object to the new view controller.
    }
    */

}
