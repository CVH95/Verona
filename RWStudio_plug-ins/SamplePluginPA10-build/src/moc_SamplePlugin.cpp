/****************************************************************************
** Meta object code from reading C++ file 'SamplePlugin.hpp'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../SamplePluginPA10/src/SamplePlugin.hpp"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#include <QtCore/qplugin.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'SamplePlugin.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_SamplePlugin_t {
    QByteArrayData data[53];
    char stringdata0[624];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SamplePlugin_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SamplePlugin_t qt_meta_stringdata_SamplePlugin = {
    {
QT_MOC_LITERAL(0, 0, 12), // "SamplePlugin"
QT_MOC_LITERAL(1, 13, 10), // "btnPressed"
QT_MOC_LITERAL(2, 24, 0), // ""
QT_MOC_LITERAL(3, 25, 5), // "timer"
QT_MOC_LITERAL(4, 31, 20), // "stateChangedListener"
QT_MOC_LITERAL(5, 52, 21), // "rw::kinematics::State"
QT_MOC_LITERAL(6, 74, 5), // "state"
QT_MOC_LITERAL(7, 80, 15), // "markerMovements"
QT_MOC_LITERAL(8, 96, 38), // "vector<rw::math::Transform3D<..."
QT_MOC_LITERAL(9, 135, 6), // "string"
QT_MOC_LITERAL(10, 142, 8), // "fileName"
QT_MOC_LITERAL(11, 151, 15), // "default_restart"
QT_MOC_LITERAL(12, 167, 11), // "Device::Ptr"
QT_MOC_LITERAL(13, 179, 3), // "dev"
QT_MOC_LITERAL(14, 183, 29), // "rw::kinematics::MovableFrame*"
QT_MOC_LITERAL(15, 213, 6), // "marker"
QT_MOC_LITERAL(16, 220, 11), // "rw::math::Q"
QT_MOC_LITERAL(17, 232, 6), // "config"
QT_MOC_LITERAL(18, 239, 29), // "rw::math::Transform3D<double>"
QT_MOC_LITERAL(19, 269, 16), // "T_marker_default"
QT_MOC_LITERAL(20, 286, 14), // "du_dvEuclidean"
QT_MOC_LITERAL(21, 301, 26), // "rw::math::Vector2D<double>"
QT_MOC_LITERAL(22, 328, 6), // "target"
QT_MOC_LITERAL(23, 335, 7), // "current"
QT_MOC_LITERAL(24, 343, 19), // "checkVelocityLimits"
QT_MOC_LITERAL(25, 363, 1), // "Q"
QT_MOC_LITERAL(26, 365, 2), // "dq"
QT_MOC_LITERAL(27, 368, 5), // "limit"
QT_MOC_LITERAL(28, 374, 7), // "delta_t"
QT_MOC_LITERAL(29, 382, 14), // "image_Jacobian"
QT_MOC_LITERAL(30, 397, 18), // "rw::math::Jacobian"
QT_MOC_LITERAL(31, 416, 1), // "z"
QT_MOC_LITERAL(32, 418, 1), // "f"
QT_MOC_LITERAL(33, 420, 8), // "imgPoint"
QT_MOC_LITERAL(34, 429, 9), // "stackJacs"
QT_MOC_LITERAL(35, 439, 35), // "vector<rw::math::Vector2D<dou..."
QT_MOC_LITERAL(36, 475, 2), // "uv"
QT_MOC_LITERAL(37, 478, 8), // "duToBase"
QT_MOC_LITERAL(38, 487, 3), // "T_0"
QT_MOC_LITERAL(39, 491, 17), // "compute_Z_image_q"
QT_MOC_LITERAL(40, 509, 15), // "Eigen::MatrixXd"
QT_MOC_LITERAL(41, 525, 6), // "Jimage"
QT_MOC_LITERAL(42, 532, 3), // "S_q"
QT_MOC_LITERAL(43, 536, 3), // "J_q"
QT_MOC_LITERAL(44, 540, 8), // "solve_dq"
QT_MOC_LITERAL(45, 549, 9), // "reference"
QT_MOC_LITERAL(46, 559, 6), // "Zimage"
QT_MOC_LITERAL(47, 566, 13), // "track_1_point"
QT_MOC_LITERAL(48, 580, 6), // "Frame*"
QT_MOC_LITERAL(49, 587, 6), // "camera"
QT_MOC_LITERAL(50, 594, 14), // "track_3_points"
QT_MOC_LITERAL(51, 609, 8), // "duvStack"
QT_MOC_LITERAL(52, 618, 5) // "du_dv"

    },
    "SamplePlugin\0btnPressed\0\0timer\0"
    "stateChangedListener\0rw::kinematics::State\0"
    "state\0markerMovements\0"
    "vector<rw::math::Transform3D<double> >\0"
    "string\0fileName\0default_restart\0"
    "Device::Ptr\0dev\0rw::kinematics::MovableFrame*\0"
    "marker\0rw::math::Q\0config\0"
    "rw::math::Transform3D<double>\0"
    "T_marker_default\0du_dvEuclidean\0"
    "rw::math::Vector2D<double>\0target\0"
    "current\0checkVelocityLimits\0Q\0dq\0limit\0"
    "delta_t\0image_Jacobian\0rw::math::Jacobian\0"
    "z\0f\0imgPoint\0stackJacs\0"
    "vector<rw::math::Vector2D<double> >\0"
    "uv\0duToBase\0T_0\0compute_Z_image_q\0"
    "Eigen::MatrixXd\0Jimage\0S_q\0J_q\0solve_dq\0"
    "reference\0Zimage\0track_1_point\0Frame*\0"
    "camera\0track_3_points\0duvStack\0du_dv"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SamplePlugin[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   89,    2, 0x08 /* Private */,
       3,    0,   90,    2, 0x08 /* Private */,
       4,    1,   91,    2, 0x08 /* Private */,
       7,    1,   94,    2, 0x08 /* Private */,
      11,    4,   97,    2, 0x08 /* Private */,
      20,    2,  106,    2, 0x08 /* Private */,
      24,    3,  111,    2, 0x08 /* Private */,
      29,    3,  118,    2, 0x08 /* Private */,
      34,    3,  125,    2, 0x08 /* Private */,
      37,    1,  132,    2, 0x08 /* Private */,
      39,    3,  135,    2, 0x08 /* Private */,
      44,    3,  142,    2, 0x08 /* Private */,
      47,    3,  149,    2, 0x08 /* Private */,
      50,    3,  156,    2, 0x08 /* Private */,
      51,    1,  163,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5,    6,
    0x80000000 | 8, 0x80000000 | 9,   10,
    QMetaType::Void, 0x80000000 | 12, 0x80000000 | 14, 0x80000000 | 16, 0x80000000 | 18,   13,   15,   17,   19,
    QMetaType::Float, 0x80000000 | 21, 0x80000000 | 21,   22,   23,
    0x80000000 | 16, 0x80000000 | 25, 0x80000000 | 25, QMetaType::Double,   26,   27,   28,
    0x80000000 | 30, QMetaType::Double, QMetaType::Double, 0x80000000 | 21,   31,   32,   33,
    0x80000000 | 30, QMetaType::Double, QMetaType::Double, 0x80000000 | 35,   31,   32,   36,
    0x80000000 | 30, 0x80000000 | 18,   38,
    0x80000000 | 40, 0x80000000 | 30, 0x80000000 | 30, 0x80000000 | 30,   41,   42,   43,
    0x80000000 | 16, 0x80000000 | 21, 0x80000000 | 21, 0x80000000 | 40,   36,   45,   46,
    0x80000000 | 21, QMetaType::Double, 0x80000000 | 48, 0x80000000 | 48,   32,   15,   49,
    0x80000000 | 35, QMetaType::Double, 0x80000000 | 48, 0x80000000 | 48,   32,   15,   49,
    0x80000000 | 30, 0x80000000 | 35,   52,

       0        // eod
};

void SamplePlugin::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SamplePlugin *_t = static_cast<SamplePlugin *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->btnPressed(); break;
        case 1: _t->timer(); break;
        case 2: _t->stateChangedListener((*reinterpret_cast< const rw::kinematics::State(*)>(_a[1]))); break;
        case 3: { vector<rw::math::Transform3D<double> > _r = _t->markerMovements((*reinterpret_cast< string(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< vector<rw::math::Transform3D<double> >*>(_a[0]) = _r; }  break;
        case 4: _t->default_restart((*reinterpret_cast< Device::Ptr(*)>(_a[1])),(*reinterpret_cast< rw::kinematics::MovableFrame*(*)>(_a[2])),(*reinterpret_cast< rw::math::Q(*)>(_a[3])),(*reinterpret_cast< rw::math::Transform3D<double>(*)>(_a[4]))); break;
        case 5: { float _r = _t->du_dvEuclidean((*reinterpret_cast< rw::math::Vector2D<double>(*)>(_a[1])),(*reinterpret_cast< rw::math::Vector2D<double>(*)>(_a[2])));
            if (_a[0]) *reinterpret_cast< float*>(_a[0]) = _r; }  break;
        case 6: { rw::math::Q _r = _t->checkVelocityLimits((*reinterpret_cast< Q(*)>(_a[1])),(*reinterpret_cast< Q(*)>(_a[2])),(*reinterpret_cast< double(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< rw::math::Q*>(_a[0]) = _r; }  break;
        case 7: { rw::math::Jacobian _r = _t->image_Jacobian((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< rw::math::Vector2D<double>(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< rw::math::Jacobian*>(_a[0]) = _r; }  break;
        case 8: { rw::math::Jacobian _r = _t->stackJacs((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< double(*)>(_a[2])),(*reinterpret_cast< vector<rw::math::Vector2D<double> >(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< rw::math::Jacobian*>(_a[0]) = _r; }  break;
        case 9: { rw::math::Jacobian _r = _t->duToBase((*reinterpret_cast< rw::math::Transform3D<double>(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< rw::math::Jacobian*>(_a[0]) = _r; }  break;
        case 10: { Eigen::MatrixXd _r = _t->compute_Z_image_q((*reinterpret_cast< rw::math::Jacobian(*)>(_a[1])),(*reinterpret_cast< rw::math::Jacobian(*)>(_a[2])),(*reinterpret_cast< rw::math::Jacobian(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< Eigen::MatrixXd*>(_a[0]) = _r; }  break;
        case 11: { rw::math::Q _r = _t->solve_dq((*reinterpret_cast< rw::math::Vector2D<double>(*)>(_a[1])),(*reinterpret_cast< rw::math::Vector2D<double>(*)>(_a[2])),(*reinterpret_cast< Eigen::MatrixXd(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< rw::math::Q*>(_a[0]) = _r; }  break;
        case 12: { rw::math::Vector2D<double> _r = _t->track_1_point((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< Frame*(*)>(_a[2])),(*reinterpret_cast< Frame*(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< rw::math::Vector2D<double>*>(_a[0]) = _r; }  break;
        case 13: { vector<rw::math::Vector2D<double> > _r = _t->track_3_points((*reinterpret_cast< double(*)>(_a[1])),(*reinterpret_cast< Frame*(*)>(_a[2])),(*reinterpret_cast< Frame*(*)>(_a[3])));
            if (_a[0]) *reinterpret_cast< vector<rw::math::Vector2D<double> >*>(_a[0]) = _r; }  break;
        case 14: { rw::math::Jacobian _r = _t->duvStack((*reinterpret_cast< vector<rw::math::Vector2D<double> >(*)>(_a[1])));
            if (_a[0]) *reinterpret_cast< rw::math::Jacobian*>(_a[0]) = _r; }  break;
        default: ;
        }
    }
}

const QMetaObject SamplePlugin::staticMetaObject = {
    { &rws::RobWorkStudioPlugin::staticMetaObject, qt_meta_stringdata_SamplePlugin.data,
      qt_meta_data_SamplePlugin,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *SamplePlugin::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SamplePlugin::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_SamplePlugin.stringdata0))
        return static_cast<void*>(const_cast< SamplePlugin*>(this));
    if (!strcmp(_clname, "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1"))
        return static_cast< rws::RobWorkStudioPlugin*>(const_cast< SamplePlugin*>(this));
    return rws::RobWorkStudioPlugin::qt_metacast(_clname);
}

int SamplePlugin::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = rws::RobWorkStudioPlugin::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 15)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 15;
    }
    return _id;
}

QT_PLUGIN_METADATA_SECTION const uint qt_section_alignment_dummy = 42;

#ifdef QT_NO_DEBUG

QT_PLUGIN_METADATA_SECTION
static const unsigned char qt_pluginMetaData[] = {
    'Q', 'T', 'M', 'E', 'T', 'A', 'D', 'A', 'T', 'A', ' ', ' ',
    'q',  'b',  'j',  's',  0x01, 0x00, 0x00, 0x00,
    0x10, 0x01, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00,
    0xfc, 0x00, 0x00, 0x00, 0x1b, 0x03, 0x00, 0x00,
    0x03, 0x00, 'I',  'I',  'D',  0x00, 0x00, 0x00,
    '*',  0x00, 'd',  'k',  '.',  's',  'd',  'u', 
    '.',  'm',  'i',  'p',  '.',  'R',  'o',  'b', 
    'w',  'o',  'r',  'k',  '.',  'R',  'o',  'b', 
    'W',  'o',  'r',  'k',  'S',  't',  'u',  'd', 
    'i',  'o',  'P',  'l',  'u',  'g',  'i',  'n', 
    '/',  '0',  '.',  '1',  0x9b, 0x0a, 0x00, 0x00,
    0x09, 0x00, 'c',  'l',  'a',  's',  's',  'N', 
    'a',  'm',  'e',  0x00, 0x0c, 0x00, 'S',  'a', 
    'm',  'p',  'l',  'e',  'P',  'l',  'u',  'g', 
    'i',  'n',  0x00, 0x00, ':',  0xa0, 0xa0, 0x00,
    0x07, 0x00, 'v',  'e',  'r',  's',  'i',  'o', 
    'n',  0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00,
    0x05, 0x00, 'd',  'e',  'b',  'u',  'g',  0x00,
    0x15, 0x12, 0x00, 0x00, 0x08, 0x00, 'M',  'e', 
    't',  'a',  'D',  'a',  't',  'a',  0x00, 0x00,
    'l',  0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
    '`',  0x00, 0x00, 0x00, 0x1b, 0x03, 0x00, 0x00,
    0x04, 0x00, 'n',  'a',  'm',  'e',  0x00, 0x00,
    0x0b, 0x00, 'p',  'l',  'u',  'g',  'i',  'n', 
    'U',  'I',  'a',  'p',  'p',  0x00, 0x00, 0x00,
    0x1b, 0x07, 0x00, 0x00, 0x07, 0x00, 'v',  'e', 
    'r',  's',  'i',  'o',  'n',  0x00, 0x00, 0x00,
    0x05, 0x00, '1',  '.',  '0',  '.',  '0',  0x00,
    0x94, 0x0a, 0x00, 0x00, 0x0c, 0x00, 'd',  'e', 
    'p',  'e',  'n',  'd',  'e',  'n',  'c',  'i', 
    'e',  's',  0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    '@',  0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
    '(',  0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
    0x80, 0x00, 0x00, 0x00, 'D',  0x00, 0x00, 0x00,
    't',  0x00, 0x00, 0x00, 'd',  0x00, 0x00, 0x00
};

#else // QT_NO_DEBUG

QT_PLUGIN_METADATA_SECTION
static const unsigned char qt_pluginMetaData[] = {
    'Q', 'T', 'M', 'E', 'T', 'A', 'D', 'A', 'T', 'A', ' ', ' ',
    'q',  'b',  'j',  's',  0x01, 0x00, 0x00, 0x00,
    0x10, 0x01, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x00,
    0xfc, 0x00, 0x00, 0x00, 0x1b, 0x03, 0x00, 0x00,
    0x03, 0x00, 'I',  'I',  'D',  0x00, 0x00, 0x00,
    '*',  0x00, 'd',  'k',  '.',  's',  'd',  'u', 
    '.',  'm',  'i',  'p',  '.',  'R',  'o',  'b', 
    'w',  'o',  'r',  'k',  '.',  'R',  'o',  'b', 
    'W',  'o',  'r',  'k',  'S',  't',  'u',  'd', 
    'i',  'o',  'P',  'l',  'u',  'g',  'i',  'n', 
    '/',  '0',  '.',  '1',  0x95, 0x0a, 0x00, 0x00,
    0x08, 0x00, 'M',  'e',  't',  'a',  'D',  'a', 
    't',  'a',  0x00, 0x00, 'l',  0x00, 0x00, 0x00,
    0x07, 0x00, 0x00, 0x00, '`',  0x00, 0x00, 0x00,
    0x1b, 0x03, 0x00, 0x00, 0x04, 0x00, 'n',  'a', 
    'm',  'e',  0x00, 0x00, 0x0b, 0x00, 'p',  'l', 
    'u',  'g',  'i',  'n',  'U',  'I',  'a',  'p', 
    'p',  0x00, 0x00, 0x00, 0x1b, 0x07, 0x00, 0x00,
    0x07, 0x00, 'v',  'e',  'r',  's',  'i',  'o', 
    'n',  0x00, 0x00, 0x00, 0x05, 0x00, '1',  '.', 
    '0',  '.',  '0',  0x00, 0x94, 0x0a, 0x00, 0x00,
    0x0c, 0x00, 'd',  'e',  'p',  'e',  'n',  'd', 
    'e',  'n',  'c',  'i',  'e',  's',  0x00, 0x00,
    0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, '@',  0x00, 0x00, 0x00,
    0x0c, 0x00, 0x00, 0x00, '(',  0x00, 0x00, 0x00,
    0x1b, 0x1a, 0x00, 0x00, 0x09, 0x00, 'c',  'l', 
    'a',  's',  's',  'N',  'a',  'm',  'e',  0x00,
    0x0c, 0x00, 'S',  'a',  'm',  'p',  'l',  'e', 
    'P',  'l',  'u',  'g',  'i',  'n',  0x00, 0x00,
    '1',  0x00, 0x00, 0x00, 0x05, 0x00, 'd',  'e', 
    'b',  'u',  'g',  0x00, ':',  0xa0, 0xa0, 0x00,
    0x07, 0x00, 'v',  'e',  'r',  's',  'i',  'o', 
    'n',  0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00,
    'D',  0x00, 0x00, 0x00, 0xc0, 0x00, 0x00, 0x00,
    0xe0, 0x00, 0x00, 0x00, 0xec, 0x00, 0x00, 0x00
};
#endif // QT_NO_DEBUG

QT_MOC_EXPORT_PLUGIN(SamplePlugin, SamplePlugin)

QT_END_MOC_NAMESPACE
