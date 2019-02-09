/**
* This file is part of the ROS package trajectory_control which belongs to the framework 3DPATROLLING. 
*
* Copyright (C) 2016-present Luigi Freda <freda at diag dot uniroma1 dot it> and Alcor Lab (La Sapienza University)
* For more information see <https://gitlab.com/luigifreda/3dpatrolling>
*
* 3DPATROLLING is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* 3DPATROLLING is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with 3DPATROLLING. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CMDVELS_H
#define CMDVELS_H


///	\class CmdVels
///	\author Luigi
///	\brief A class for managing [v,omega] to [vl,vr] conversions by taking into account also steering efficiency 
///	\note 
/// 	\todo 
///	\date
///	\warning
class CmdVels
{
public:
    CmdVels(double d=0.397, double chi=1.):v_(0.),omega_(0.),vl_(0.),vr_(0.)
    {
        set(d,chi); 
    }
    
public: // setters 
    void set(double d, double chi)
    {
        d_ = d; chi_ = chi; 
    }
    
    void setChi(double val) {chi_ = val; }
    void setD(double val) {d_ = val; }
    
    void updateVlVr()
    {
        //[vl vr] = Tv_inv [v omega] 
        //Tv = [1,-(d/(2*chi));
        //      1, (d/(2*chi))]
        double d_over_2_chi = 0.5*d_/chi_; 
        vl_ = v_ - d_over_2_chi*omega_;
        vr_ = v_ + d_over_2_chi*omega_;
    }
    
    void updateVOmega()
    {
        //[v omega] = Tv_inv [vl vr]
        //Tv_inv = [    1/2,   1/2]
        //         [ -chi/d, chi/d]
        double chi_over_d = chi_/d_;
        v_     = 0.5*(vl_+vr_);
        omega_ = chi_over_d*(-vl_+vr_);
    }
    
public: // getters 
    
    double getV() const {return v_;} 
    double getOmega() const {return omega_;}
    double getVl() const {return vl_;}
    double getVr() const {return vr_;}
    
public: 
    double v_, omega_; // linear velocity and angular velocity 
    double vl_, vr_;   // left velocity and right velocity 
    
    double d_;   // axis width
    double chi_; // steering efficiency 
};


#endif /* CMDVELS_H */

