#pragma  once

#include "Exception.h"
#include <limits>
#include <iomanip>
#include <sstream>
#include <cstdlib>
#include <cmath>
#include "Const.h"

using namespace std;

class TimeSystem {
public:
    // list of time systems supported by this class
    enum SystemType {
        Unknown = 0,
        GPS, // GPS system time
        GLO, // glonass system time
        GAL, // Galileo system time
        BDT, // BDS system time
        QZS, // qzss time
        UTC, // utc
        TAI,
        TT,
        count // the number of systems
    };

    // Constructor, including empty constructor
    TimeSystem(const SystemType sys = GPS) { //NOLINT
        if (sys >= count)
            system = GPS;
        else
            system = sys;
    }

    // Constructor
    explicit TimeSystem(const std::string &str) {
        for (int i = 0; i < count; i++) {
            if (sys_strings[i] == str) {
                system = static_cast<SystemType>(i);
                break;
            }
        }
    }

    bool operator==(const TimeSystem &right) const {
        if (system == right.system)
            return true;
        return false;
    }

    bool operator!=(const TimeSystem &right) const {
        return !operator==(right);
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << sys_strings[static_cast<int>(system)];
        return oss.str();
    }

    struct epoch_params {
        int Nbits;
        int bitmask;
        long MJDEpoch;
    };

    const epoch_params &getParams() const {
        return EPOCH_PARAM[static_cast<int>(system)];
    }

    // time system
    SystemType system;
    // string for enum sys, 1 vs 1
    static const std::string sys_strings[];
    static const epoch_params EPOCH_PARAM[];
}; // end class TimeSystem


inline std::ostream &operator<<(std::ostream &os, const TimeSystem &ts) {
    return os << ts.toString() << std::endl;
}

/**
 *  定义一个通用时间的目的是为了实现不同历法之间的快速转换；
 *  考虑到计算精度和计算方便，需要把时间分为day和second of day两个变量；
 *  这是因为，如果用一个second of day作为唯一变量，受到计算机存储字长的影响，
 *  在时间转换时，精度会下降，难以满足高精度需求。
 */
class CommonTime {
public:
    explicit CommonTime(const long day = 0, const double sod = 0.0, const TimeSystem timeSystem = TimeSystem::GPS) {
        m_day = day;
        m_sod = sod;
        m_timeSystem = timeSystem;
    }

    CommonTime(const CommonTime &right) = default;

    // Destructor.
    virtual ~CommonTime() = default;

    void set(const long day, const double sod = 0.0, const TimeSystem timeSystem = TimeSystem::GPS) {
        m_day = day;
        m_sod = sod;
        m_timeSystem = timeSystem;
    }

    void setTimeSystem(const TimeSystem &timeSystem) {
        m_timeSystem = timeSystem;
    }

    void get(long &day, double &sod, TimeSystem &timeSystem) const {
        day = m_day;
        sod = m_sod;
        timeSystem = m_timeSystem;
    }

    CommonTime &operator=(const CommonTime &right) = default;

    double operator-(const CommonTime &right) const {
        if (m_timeSystem != right.m_timeSystem) {
            throw InvalidRequest("CommonTime objects not in same time system, cannot be different");
        }
        return SEC_PER_DAY * static_cast<double>(m_day - right.m_day) + m_sod - right.m_sod;
    }

    CommonTime operator+(const double sec) const {
        return CommonTime(*this).addSeconds(sec);
    }

    CommonTime &operator+=(const double sec) {
        addSeconds(sec);
        return *this;
    }

    CommonTime operator-(const double sec) const {
        CommonTime tempTime = *this;
        tempTime.addSeconds(-sec);
        return tempTime;
    }

    CommonTime &operator-=(const double seconds) {
        addSeconds(-seconds);
        return *this;
    }

    CommonTime &addSeconds(double seconds) {
        long days = 0;

        if (abs(seconds) >= SEC_PER_DAY) {
            days = static_cast<long>(seconds * DAY_PER_SEC);
            seconds -= days * SEC_PER_DAY;
        }
        m_day += days;
        m_sod += seconds;
        normalize();
        return *this;
    }

    // 归化
    bool normalize() {
        if (abs(m_sod) >= SEC_PER_DAY) {
            const auto day = static_cast<long>(m_sod) / SEC_PER_DAY;
            m_day += day;
            m_sod -= day * SEC_PER_DAY;
        }
        if (m_sod < 0) {
            m_sod = m_sod + SEC_PER_DAY;
            --m_day;
        }
        return true;
    }

    std::string toString() const {
        ostringstream oss;
        oss << setfill('0')
                << setw(7) << m_day << " "
                << fixed << setprecision(15) << setw(17) << m_sod
                << " " << m_timeSystem.toString();
        return oss.str();
    }

    bool operator==(const CommonTime &right) const {
        if (m_timeSystem != right.m_timeSystem)
            return false;
        if (m_day == right.m_day && fabs(m_sod - right.m_sod) < std::numeric_limits<double>::epsilon())
            return true;
        return false;
    }

    bool operator!=(const CommonTime &right) const {
        return !operator==(right);
    }

    bool operator<(const CommonTime &right) const {
        if (m_timeSystem != right.m_timeSystem) {
            throw InvalidRequest("CommonTime objects not in same time system, cannot be compared: "
                                 + m_timeSystem.toString() +
                                 " != " + right.m_timeSystem.toString());
        }

        if (m_day < right.m_day)
            return true;
        if (m_day > right.m_day)
            return false;

        if (m_sod < right.m_sod)
            return true;
        return false;
    }

    bool operator>(const CommonTime &right) const {
        return !operator<=(right);
    }

    bool operator<=(const CommonTime &right) const {
        return operator<(right) || operator==(right);
    }

    bool operator>=(const CommonTime &right) const {
        return !operator<(right);
    }

    long m_day{}; // Modified Julian Day
    double m_sod{}; // seconds-of-day
    TimeSystem m_timeSystem;
}; // end class CommonTime

// 'julian day' of earliest epoch expressible by CommonTime; 1/1/4713 B.C.
static const auto BEGINNING_OF_TIME = CommonTime(0L, 0.0, TimeSystem::GPS);

// 'julian day' of latest 'julian day' expressible by CommonTime,
// 1/1/4713 A.D.
static const auto END_OF_TIME = CommonTime(3442448L, 0.0, TimeSystem::GPS);

inline std::ostream &operator<<(std::ostream &os, const CommonTime &ct) {
    os << ct.toString();
    return os;
}

// -----------------------------------
// CivilTime
// -----------------------------------
class CivilTime {
public:
    CivilTime(int yr = 0,
              int mo = 0,
              int dy = 0,
              int hr = 0,
              int mn = 0,
              double s = 0.0,
              TimeSystem ts = TimeSystem::GPS)
        : year(yr), month(mo), day(dy), hour(hr), minute(mn), second(s) {
        timeSys = ts;
    }

    /**
     * Copy Constructor.
     */
    CivilTime(const CivilTime &right)
        : year(right.year), month(right.month), day(right.day),
          hour(right.hour), minute(right.minute), second(right.second) {
        timeSys = right.timeSys;
    }

    CivilTime &operator=(const CivilTime &right);

    bool operator==(const CivilTime &right) const;

    std::string toString() const {
        std::ostringstream oss;
        oss << setw(4) << year << "/"
                << setw(2) << month << "/"
                << setw(2) << day << " "
                << setw(2) << hour << ":"
                << setw(2) << minute << ":"
                << setw(2) << second << " "
                << timeSys.toString();

        return oss.str();
    };

    /// Virtual Destructor.
    virtual ~CivilTime() {
    }

    int year;
    int month;
    int day;
    int hour;
    int minute;
    double second;

    TimeSystem timeSys;
};

inline std::ostream &operator<<(std::ostream &s, const CivilTime &cit) {
    s << cit.toString();
    return s;
}

class JulianDate {
public:
    JulianDate(long double j = 0., TimeSystem ts = TimeSystem::GPS)
        : jd(j) { timeSystem = ts; }

    JulianDate(const JulianDate &right)
        : jd(right.jd) { timeSystem = right.timeSystem; }

    JulianDate &operator=(const JulianDate &right);

    string toString() {
        std::ostringstream oss;
        oss << fixed << setw(16) << jd << ":"
                << timeSystem.toString();

        return oss.str();
    }

    /// Virtual Destructor.
    virtual ~JulianDate() {
    }

    void reset();

    bool operator==(const JulianDate &right) const;

    bool operator!=(const JulianDate &right) const;

    bool operator<(const JulianDate &right) const;

    long double jd;

    TimeSystem timeSystem;
};

inline std::ostream &operator<<(std::ostream &s, JulianDate &jd) {
    s << jd.toString();
    return s;
}

// year/day_of_year/second_of_day
// 方便用于输出单天解
class YDSTime {
public:
    YDSTime(int y = 0, int d = 0, double s = 0.0,
            TimeSystem ts = TimeSystem::GPS) {
        year = y;
        doy = d;
        sod = s;
        timeSystem = ts;
    }

    YDSTime(const YDSTime &right)
        : year(right.year), doy(right.doy), sod(right.sod) { timeSystem = right.timeSystem; }

    YDSTime &operator=(const YDSTime &right);

    std::string toString() const {
        std::ostringstream oss;
        oss << setw(4) << year << " "
                << setw(3) << doy << " "
                << setw(14) << sod << " "
                << timeSystem.toString() << " ";

        return oss.str();
    };

    // Virtual Destructor.
    virtual ~YDSTime() {
    }

    virtual void reset();

    bool operator==(const YDSTime &right) const;

    bool operator!=(const YDSTime &right) const;

    bool operator<(const YDSTime &right) const;

    int year;
    int doy;
    double sod;
    TimeSystem timeSystem;
};

inline std::ostream &operator<<(std::ostream &s, const YDSTime &yt) {
    s << yt.toString();
    return s;
};

class MJD {
public:
    MJD(long double m = 0.,
        TimeSystem ts = TimeSystem::GPS)
        : mjd(m) { timeSystem = ts; }


    MJD(const MJD &right)
        : mjd(right.mjd) { timeSystem = right.timeSystem; }


    MJD &operator=(const MJD &right);

    /// Virtual Destructor.
    virtual ~MJD() {
    }

    void reset();

    std::string toString() {
        std::ostringstream oss;
        oss << setw(8) << mjd
                << timeSystem.toString();

        return oss.str();
    };

    bool operator==(const MJD &right) const;

    bool operator!=(const MJD &right) const;

    bool operator<(const MJD &right) const;

    bool operator>(const MJD &right) const;

    bool operator<=(const MJD &right) const;

    bool operator>=(const MJD &right) const;

    long double mjd;
    TimeSystem timeSystem;
};

inline std::ostream &operator<<(std::ostream &s, MJD &mjd) {
    s << mjd.toString();
    return s;
}

class JD2020 {
public:
    explicit JD2020(const long double m = 0, const TimeSystem ts = TimeSystem::GPS)
        : jd(m) {
        timeSystem = ts;
    }


    JD2020(const JD2020 &right)
        : jd(right.jd) { timeSystem = right.timeSystem; }


    JD2020 &operator=(const JD2020 &right);

    /// Virtual Destructor.
    virtual ~JD2020() = default;

    void reset();

    std::string toString() const {
        std::ostringstream oss;
        oss << setw(8) << jd
                << timeSystem.toString();

        return oss.str();
    };

    bool operator==(const JD2020 &right) const;

    bool operator!=(const JD2020 &right) const;

    bool operator<(const JD2020 &right) const;

    bool operator>(const JD2020 &right) const;

    bool operator<=(const JD2020 &right) const;

    bool operator>=(const JD2020 &right) const;

    long double jd;
    TimeSystem timeSystem;
};

inline std::ostream &operator<<(std::ostream &s, JD2020 &jd) {
    s << jd.toString();
    return s;
}


/// This class encapsulates the "Full Week and Seconds-of-week"
/// time representation.
class WeekSecond {
public:
    WeekSecond(const unsigned int w = 0, const double s = 0, const TimeSystem ts = TimeSystem::GPS) : week(w), sow(s) { //NOLINT
        timeSystem = ts;
    }

    WeekSecond &operator=(const WeekSecond &right);

    /// Virtual Destructor.
    virtual ~WeekSecond() = default;

    int Nbits() const {
        return timeSystem.getParams().Nbits;
    }

    int bitmask() const {
        return timeSystem.getParams().bitmask;
    }

    long MJDEpoch() const {
        return timeSystem.getParams().MJDEpoch;
    }

    int rollover() const { return bitmask() + 1; }


    unsigned int getDayOfWeek() const {
        return static_cast<unsigned int>(sow) / SEC_PER_DAY;
    }

    double getSOW() const { return sow; }

    void reset();

    bool operator==(const WeekSecond &right) const;

    bool operator!=(const WeekSecond &right) const;

    bool operator<(const WeekSecond &right) const;

    bool operator>(const WeekSecond &right) const;

    bool operator<=(const WeekSecond &right) const;

    bool operator>=(const WeekSecond &right) const;

    double operator-(const WeekSecond &right) const;


    void setEpoch(const unsigned int e) {
        week &= bitmask();
        week |= e << Nbits();
    }

    void setModWeek(const unsigned int w) {
        week &= ~bitmask();
        week |= w & bitmask();
    }

    void setEpochModWeek(const unsigned int e, const unsigned int w) {
        setEpoch(e);
        setModWeek(w);
    }

    unsigned int getWeek() const {
        return week;
    }

    unsigned int getModWeek() const {
        return (week & bitmask());
    }

    unsigned int getEpoch() const {
        return (week >> Nbits());
    }

    void getEpochModWeek(unsigned int &e, unsigned int &w) const {
        e = getEpoch();
        w = getModWeek();
    }

    std::string toString() const {
        std::ostringstream oss;
        oss << setw(4) << week
                << setw(10) << sow
                << timeSystem.toString();

        return oss.str();
    }

    //@}
    unsigned int week;
    double sow;
    TimeSystem timeSystem;
};

inline std::ostream &operator<<(std::ostream &s, const WeekSecond &ws) {
    s << ws.toString();
    return s;
}
