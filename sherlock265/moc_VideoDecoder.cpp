/****************************************************************************
** Meta object code from reading C++ file 'VideoDecoder.hh'
**
** Created: Wed Oct 16 16:30:46 2013
**      by: The Qt Meta Object Compiler version 63 (Qt 4.8.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "VideoDecoder.hh"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'VideoDecoder.hh' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_VideoDecoder[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
      10,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      14,   13,   13,   13, 0x05,

 // slots: signature, parameters, type, tag, flags
      36,   13,   13,   13, 0x0a,
      51,   13,   13,   13, 0x0a,
      65,   13,   13,   13, 0x0a,
      90,   85,   13,   13, 0x0a,
     115,   85,   13,   13, 0x0a,
     140,   85,   13,   13, 0x0a,
     165,   85,   13,   13, 0x0a,
     189,   85,   13,   13, 0x0a,
     210,   85,   13,   13, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_VideoDecoder[] = {
    "VideoDecoder\0\0displayImage(QImage*)\0"
    "startDecoder()\0stopDecoder()\0"
    "singleStepDecoder()\0flag\0"
    "showCBPartitioning(bool)\0"
    "showTBPartitioning(bool)\0"
    "showPBPartitioning(bool)\0"
    "showIntraPredMode(bool)\0showPBPredMode(bool)\0"
    "showDecodedImage(bool)\0"
};

void VideoDecoder::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        VideoDecoder *_t = static_cast<VideoDecoder *>(_o);
        switch (_id) {
        case 0: _t->displayImage((*reinterpret_cast< QImage*(*)>(_a[1]))); break;
        case 1: _t->startDecoder(); break;
        case 2: _t->stopDecoder(); break;
        case 3: _t->singleStepDecoder(); break;
        case 4: _t->showCBPartitioning((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->showTBPartitioning((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->showPBPartitioning((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: _t->showIntraPredMode((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 8: _t->showPBPredMode((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 9: _t->showDecodedImage((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData VideoDecoder::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject VideoDecoder::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_VideoDecoder,
      qt_meta_data_VideoDecoder, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &VideoDecoder::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *VideoDecoder::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *VideoDecoder::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_VideoDecoder))
        return static_cast<void*>(const_cast< VideoDecoder*>(this));
    return QThread::qt_metacast(_clname);
}

int VideoDecoder::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 10)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 10;
    }
    return _id;
}

// SIGNAL 0
void VideoDecoder::displayImage(QImage * _t1)
{
    void *_a[] = { 0, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}
QT_END_MOC_NAMESPACE
