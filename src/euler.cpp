#include "euler.h"

Euler::Euler()
{

}

Euler::~Euler()
{
}

Eigen::Vector4i Euler::axes2vector(std::string axes){
    Eigen::Vector4i result;
    const std::unordered_map<std::string,std::function<void(Eigen::Vector4i&)>> AXESMAP{
                {"sxyz",   [](Eigen::Vector4i& result){ result<<0, 0, 0, 0; }},
                {"sxyx",   [](Eigen::Vector4i& result){ result<<0, 0, 1, 0; }},
                {"sxzy",   [](Eigen::Vector4i& result){ result<<0, 1, 0, 0; }},
                {"sxzx",   [](Eigen::Vector4i& result){ result<<0, 1, 1, 0; }},
                {"syzx",   [](Eigen::Vector4i& result){ result<<1, 0, 0, 0; }},
                {"syzy",   [](Eigen::Vector4i& result){ result<<1, 0, 1, 0; }},
                {"syxz",   [](Eigen::Vector4i& result){ result<<1, 1, 0, 0; }},
                {"syxy",   [](Eigen::Vector4i& result){ result<<1, 1, 1, 0; }},
                {"szxy",   [](Eigen::Vector4i& result){ result<<2, 0, 0, 0; }},
                {"szxz",   [](Eigen::Vector4i& result){ result<<2, 0, 1, 0; }},
                {"szyx",   [](Eigen::Vector4i& result){ result<<2, 1, 0, 0; }},
                {"szyz",   [](Eigen::Vector4i& result){ result<<2, 1, 1, 0; }},
                {"rzyx",   [](Eigen::Vector4i& result){ result<<0, 0, 0, 1; }},
                {"rxyx",   [](Eigen::Vector4i& result){ result<<0, 0, 1, 1; }},
                {"ryzx",   [](Eigen::Vector4i& result){ result<<0, 1, 0, 1; }},
                {"rxzx",   [](Eigen::Vector4i& result){ result<<0, 1, 1, 1; }},
                {"rxzy",   [](Eigen::Vector4i& result){ result<<1, 0, 0, 1; }},
                {"ryzy",   [](Eigen::Vector4i& result){ result<<1, 0, 1, 1; }},
                {"rzxy",   [](Eigen::Vector4i& result){ result<<1, 1, 0, 1; }},
                {"ryxy",   [](Eigen::Vector4i& result){ result<<1, 1, 1, 1; }},
                {"ryxz",   [](Eigen::Vector4i& result){ result<<2, 0, 0, 1; }},
                {"rzxz",   [](Eigen::Vector4i& result){ result<<2, 0, 1, 1; }},
                {"rxyz",   [](Eigen::Vector4i& result){ result<<2, 1, 0, 1; }},
                {"rzyz",   [](Eigen::Vector4i& result){ result<<2, 1, 1, 1; }},
            };
    const auto end = AXESMAP.end();
    auto it = AXESMAP.find(axes);
    if (it != end) {
        it->second(result);
    } else {
        result<<-1,-1,-1,-1;
        std::cerr<<"no match axes string!!!!"<<std::endl;
    }
    return result;
}

Eigen::Matrix3f Euler::euler2mat(float ai, float aj, float ak, std::string axes){
//    """Return rotation matrix from Euler angles and axis sequence.
//    Parameters
//    ----------
//    ai : float
//        First rotation angle (according to `axes`).
//    aj : float
//        Second rotation angle (according to `axes`).
//    ak : float
//        Third rotation angle (according to `axes`).
//    axes : str, optional
//        Axis specification; one of 24 axis sequences as string or encoded
//        tuple - e.g. ``sxyz`` (the default).
//    Returns
//    -------
//    mat : array-like shape (3, 3) or (4, 4)
//        Rotation matrix or affine.
//    Examples
//    --------
//    >>> R = euler2mat(1, 2, 3, 'syxz')
//    >>> np.allclose(np.sum(R[0]), -1.34786452)
//    True
//    >>> R = euler2mat(1, 2, 3, (0, 1, 0, 1))
//    >>> np.allclose(np.sum(R[0]), -0.383436184)
//    True
//    """
    Eigen::Vector4i next_axis;
    next_axis<<1,2,0,1;
    Eigen::Vector4i axisVec = axes2vector(axes) ;//firstaxis, parity, repetition, frame
    int firstaxis=axisVec(0);
    int parity=axisVec(1);
    int repetition=axisVec(2);
    int frame=axisVec(3);
    int i = firstaxis;
    int j = next_axis(i+parity);
    int k = next_axis(i-parity+1);

    if (frame!=0){
        float tmp=ai;
        ai=ak;
        ak=tmp;
    }
    if (parity!=0){
        ai=-ai;
        aj=-aj;
        ak=-ak;
    }

    float si, sj, sk; si= sin(ai); sj=sin(aj); sk=sin(ak);
    float ci, cj, ck; ci= cos(ai); cj=cos(aj); ck=cos(ak);
    float cc, cs; cc = ci*ck; cs= ci*sk;
    float sc, ss; sc = si*ck; ss= si*sk;

    Eigen::Matrix3f M= Eigen::MatrixXf::Identity(3,3);
    if (repetition!=0){
        M(i, i) = cj;
        M(i, j) = sj*si;
        M(i, k) = sj*ci;
        M(j, i) = sj*sk;
        M(j, j) = -cj*ss+cc;
        M(j, k) = -cj*cs-sc;
        M(k, i) = -sj*ck;
        M(k, j) = cj*sc+cs;
        M(k, k) = cj*cc-ss;
    }
    else{
        M(i, i) = cj*ck;
        M(i, j) = sj*sc-cs;
        M(i, k) = sj*cc+ss;
        M(j, i) = cj*sk;
        M(j, j) = sj*ss+cc;
        M(j, k) = sj*cs-sc;
        M(k, i) = -sj;
        M(k, j) = cj*si;
        M(k, k) = cj*ci;
    }
    return M;
}

Eigen::Vector3f Euler::mat2euler(Eigen::Matrix3f &M, std::string axes){
//    """Return Euler angles from rotation matrix for specified axis sequence.
//    Note that many Euler angle triplets can describe one matrix.
//    Parameters
//    ----------
//    mat : array-like shape (3, 3) or (4, 4)
//        Rotation matrix or affine.
//    axes : str, optional
//        Axis specification; one of 24 axis sequences as string or encoded
//        tuple - e.g. ``sxyz`` (the default).
//    Returns
//    -------
//    ai : float
//        First rotation angle (according to `axes`).
//    aj : float
//        Second rotation angle (according to `axes`).
//    ak : float
//        Third rotation angle (according to `axes`).
//    Examples
//    --------
//    >>> R0 = euler2mat(1, 2, 3, 'syxz')
//    >>> al, be, ga = mat2euler(R0, 'syxz')
//    >>> R1 = euler2mat(al, be, ga, 'syxz')
//    >>> np.allclose(R0, R1)
//    True
//    """
    Eigen::Vector4i next_axis;
    next_axis<<1,2,0,1;
    Eigen::Vector4i axisVec = axes2vector(axes) ;//firstaxis, parity, repetition, frame
    int firstaxis=axisVec(0);
    int parity=axisVec(1);
    int repetition=axisVec(2);
    int frame=axisVec(3);

    int i = firstaxis;
    int j = next_axis(i+parity);
    int k = next_axis(i-parity+1);

    float ax,ay,az;
    if (repetition!=0)
    {
        float sy = sqrt(M(i, j)*M(i, j) + M(i, k)*M(i, k));
        if (sy > _EPS4)
        {
            ax = atan2( M(i, j),  M(i, k));
            ay = atan2( sy,       M(i, i));
            az = atan2( M(j, i), -M(k, i));
        }
        else
        {
            ax = atan2(-M(j, k),  M(j, j));
            ay = atan2( sy,       M(i, i));
            az = 0.0;
        }
    }
    else
    {
        float cy = sqrt(M(i, i)*M(i, i) + M(j, i)*M(j, i));
        if (cy > _EPS4)
        {
            ax = atan2( M(k, j),  M(k, k));
            ay = atan2(-M(k, i),  cy);
            az = atan2( M(j, i),  M(i, i));
        }
        else
        {
            ax = atan2(-M(j, k),  M(j, j));
            ay = atan2(-M(k, i),  cy);
            az = 0.0;
        }
     }

    if (parity!=0)
    {
        ax=-ax;
        ay=-ay;
        az=-az;
    }
    if (frame!=0)
    {
        float tmp=ax;
        ax=az;
        az=tmp;
    }
    Eigen::Vector3f res; res<<ax,ay,az;
    return res;
}

Eigen::Quaternionf Euler::euler2quat(float ai, float aj, float ak, std::string axes){
  Eigen::Matrix3f mat=euler2mat(ai,aj, ak,axes);
  Eigen::Quaternionf q(mat);
  return q;
}

Eigen::Vector3f Euler::quat2euler(Eigen::Quaternionf q,std::string axes){
    Eigen::Matrix3f rot = q.toRotationMatrix();
    return mat2euler(rot, axes);
}

