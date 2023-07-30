#include "Ipc.hpp"

Ipc::Server ipcServer;
std::shared_ptr<ControllerDevice> fakemove_;

int main(int argc, char *argv[])
{
    ipcServer.init("ApriltagPipeIn");

    char buffer[1024];

    for (;;) 
    {
        Ipc::Connection ipcConnection = ipcServer.accept();

        if (ipcConnection.recv(buffer, sizeof(buffer)))
        {
            std::string rec = buffer;
            std::istringstream iss(rec);
            std::string word;

            std::string s = "";

            while (iss >> word)
            {
                if (word == "addhipmove")
                {
                    if (fakemove_ != nullptr)
                    {
                        s = s + " alreadyadded";
                    }
                    else
                    {
                        fakemove_ = std::make_shared<ControllerDevice>("Example_ControllerDevice", ControllerDevice::Handedness::ANY);
                        this->AddDevice(fakemove_);

                        s = s + " added";
                    }
                }
                else if (word == "hipmoveinput")
                {
                    if (fakemove_ == nullptr)
                    {
                        s = s + " notspawned";
                    }
                    else
                    {
                        float x, y, rx, ry, a, b;
                        iss >> x; iss >> y; iss >> rx; iss >> ry; iss >> a; iss >> b;

                        fakemove_->SetDirection(x, y, rx, ry, a, b);

                        s = s + " updated";
                    }
                }
                else if (word == "addtracker")
                {
                    //MessageBoxA(NULL, word.c_str(), "Example Driver", MB_OK);
                    std::string name, role;

                    iss >> name;
                    iss >> role;

                    if (name == "")
                    {
                        name = "UnnamedTracker" + std::to_string(this->trackers_.size());
                        role = "TrackerRole_Waist";        //should be "vive_tracker_left_foot" or "vive_tracker_left_foot" or "vive_tracker_waist"
                    }

                    auto addtracker = std::make_shared<TrackerDevice>(name, role);
                    this->AddDevice(addtracker);
                    this->trackers_.push_back(addtracker);
                    addtracker->reinit(tracker_max_saved, tracker_max_time, tracker_smoothing);
                    s = s + " added";
                }
                else if (word == "addstation")
                {
                    auto addstation = std::make_shared<TrackingReferenceDevice>("AprilCamera" + std::to_string(this->devices_.size()));
                    this->AddDevice(addstation);
                    this->stations_.push_back(addstation);
                    s = s + " added";
                }
                else if (word == "updatestation")
                {
                    int idx;
                    double a, b, c, qw, qx, qy, qz;
                    iss >> idx; iss >> a; iss >> b; iss >> c; iss >> qw; iss >> qx; iss >> qy; iss >> qz;

                    if (idx < this->stations_.size())
                    {
                        this->stations_[idx]->UpdatePose(a, b, c, qw, qx, qy, qz);
                        s = s + " updated";
                    }
                    else
                    {
                        s = s + " idinvalid";
                    }

                }
                else if (word == "synctime")
                {
                    std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
                    s = s + " " + std::to_string(this->frame_timing_avg_);
                    s = s + " " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(now - this->last_frame_time_).count());
                }
                else if (word == "updatepose")
                {
                    int idx;
                    double a, b, c, qw, qx, qy, qz, time, smoothing;
                    iss >> idx; iss >> a; iss >> b; iss >> c; iss >> qw; iss >> qx; iss >> qy; iss >> qz; iss >> time; iss >> smoothing;

                    if (idx < this->trackers_.size())
                    {
                        if(time < 0)
                            time = -time;

                        double pose[7];
                        int statuscode = this->trackers_[idx]->get_next_pose(time, pose);

                        this->trackers_[idx]->save_current_pose(a, b, c, qw, qx, qy, qz, time);
                        //this->trackers_[idx]->UpdatePos(a, b, c, time, 1-smoothing);
                        //this->trackers_[idx]->UpdateRot(qw, qx, qy, qz, time, 1-smoothing);

                        //this->trackers_[idx]->Update();

                        s = s + " updated " + std::to_string(idx);

                        s = s + " " + std::to_string(pose[0]) +
                            " " + std::to_string(pose[1]) +
                            " " + std::to_string(pose[2]) +
                            " " + std::to_string(pose[3]) +
                            " " + std::to_string(pose[4]) +
                            " " + std::to_string(pose[5]) +
                            " " + std::to_string(pose[6]) +
                            " " + std::to_string(statuscode);

                    }
                    else
                    {
                        s = s + " idinvalid";
                    }

                }
                /*                                      no longer supported by new smoothing
                else if (word == "updatepos")
                {
                    int idx;
                    double a, b, c, time, smoothing;
                    iss >> idx; iss >> a; iss >> b; iss >> c; iss >> time; iss >> smoothing;

                    if (idx < this->devices_.size())
                    {
                        this->trackers_[idx]->UpdatePos(a, b, c, time, smoothing);
                        this->trackers_[idx]->Update();
                        s = s + " updated";
                    }
                    else
                    {
                        s = s + " idinvalid";
                    }

                }
                else if (word == "updaterot")
                {
                    int idx;
                    double qw, qx, qy, qz, time, smoothing;
                    iss >> qw; iss >> qx; iss >> qy; iss >> qz; iss >> time; iss >> smoothing;

                    if (idx < this->devices_.size())
                    {
                        this->trackers_[idx]->UpdateRot(qw, qx, qy, qz, time, smoothing);
                        this->trackers_[idx]->Update();
                        s = s + " updated";
                    }
                    else
                    {
                        s = s + " idinvalid";
                    }

                }*/
                else if (word == "getdevicepose")
                {
                    int idx;
                    iss >> idx;

                    vr::TrackedDevicePose_t hmd_pose[10];
                    vr::VRServerDriverHost()->GetRawTrackedDevicePoses(1, hmd_pose, 10);

                    vr::HmdQuaternion_t q = GetRotation(hmd_pose[idx].mDeviceToAbsoluteTracking);
                    vr::HmdVector3_t pos = GetPosition(hmd_pose[idx].mDeviceToAbsoluteTracking);

                    s = s + " devicepose " + std::to_string(idx);
                    s = s + " " + std::to_string(pos.v[0]) +
                        " " + std::to_string(pos.v[1]) +
                        " " + std::to_string(pos.v[2]) +
                        " " + std::to_string(q.w) +
                        " " + std::to_string(q.x) +
                        " " + std::to_string(q.y) +
                        " " + std::to_string(q.z);
                }
                else if (word == "gettrackerpose")
                {
                    int idx;
                    double time_offset;
                    iss >> idx;
                    iss >> time_offset;

                    //                    if (idx == 0)
                    //                        Log(">----------------");

                    if (idx < this->devices_.size())
                    {
                        s = s + " trackerpose " + std::to_string(idx);

                        double pose[7];
                        int statuscode = this->trackers_[idx]->get_next_pose(time_offset, pose);

                        s = s + " " + std::to_string(pose[0]) +
                            " " + std::to_string(pose[1]) +
                            " " + std::to_string(pose[2]) +
                            " " + std::to_string(pose[3]) +
                            " " + std::to_string(pose[4]) +
                            " " + std::to_string(pose[5]) +
                            " " + std::to_string(pose[6]) +
                            " " + std::to_string(statuscode);
                    }
                    else
                    {
                        s = s + " idinvalid";
                    }

                }
                else if (word == "numtrackers")
                {
                    s = s + " numtrackers " + std::to_string(this->trackers_.size()) + " 0.5.4";
                }
                else if (word == "settings")
                {
                    int msaved;
                    double mtime;
                    double msmooth;
                    iss >> msaved;
                    iss >> mtime;
                    iss >> msmooth;
                    
                    for (auto& device : this->trackers_)
                        device->reinit(msaved,mtime,msmooth);

                    tracker_max_saved = msaved;
                    tracker_max_time = mtime;
                    tracker_smoothing = msmooth;

                    s = s + "  changed";
                }
                else
                {
                    s = s + "  unrecognized";
                }
            }

            s = s + "  OK\0";

            // = length of string + terminating '\0' !!!
            ipcConnection.send(s.c_str(), (s.length() + 1));
        }
        /*
        }
        else
        {
            Sleep(1);
        }
        */
    }
    return 0;
}
