#include "emboxvccursor.h"
#include <QtCore/qdebug.h>

QT_BEGIN_NAMESPACE

static unsigned char *__calculateCursorLocation(struct fb_info *fb, int x, int y);

QEmboxCursor::QEmboxCursor()
: mouseX(0), mouseY(0), cursor_H(20), cursor_W(10), inited(0)
{
	cursor = QImage(QSize(cursor_W, cursor_H), QImage::Format_RGB16);
	cursor.fill(Qt::red);

	dirtyRect = new unsigned char[cursor_H * cursor_W];
}

QEmboxCursor::~QEmboxCursor() {
	delete [] dirtyRect;
}

void QEmboxCursor::emboxCursorRedraw(struct fb_info *fb, int x, int y) {
	if (inited) {
		flushDirtyRect(fb, __calculateCursorLocation(fb, mouseX, mouseY));
	} else {
		inited = 1;
	}
	storeDirtyRect(fb,  __calculateCursorLocation(fb, x, y));
	drawCursor(fb, __calculateCursorLocation(fb, x, y));

	mouseX = x;
	mouseY = y;
}

static unsigned char *__calculateCursorLocation(struct fb_info *fb, int x, int y) {
	return fb->screen_base + (y * fb->var.xres + x) * 2;
}

void QEmboxCursor::drawCursor(struct fb_info *fb, unsigned char *begin) {
    int shift, i;

    int bpp = fb->var.bits_per_pixel / 8;

    for (i = 0, shift = 0; i < cursor.height(); i++ , shift += fb->var.xres * bpp) {
    	memcpy(begin + shift, (const void *)cursor.constScanLine(i), cursor.bytesPerLine());
    }
}

void QEmboxCursor::storeDirtyRect(struct fb_info *fb, unsigned char *begin) {
    int shift, i;

    int bpp = fb->var.bits_per_pixel / 8;

    for (i = 0, shift = 0; i < cursor_H; i++ , shift += fb->var.xres * bpp) {
    	memcpy(dirtyRect + i * cursor_W, (const void *)(begin + shift), cursor_W);
    }
}

void QEmboxCursor::flushDirtyRect(struct fb_info *fb, unsigned char *begin) {
    int shift, i;

    int bpp = fb->var.bits_per_pixel / 8;

    for (i = 0, shift = 0; i < cursor_H; i++, shift += fb->var.xres * bpp) {
    	memcpy(begin + shift, (const void *)(dirtyRect + i * cursor_W), cursor_W);
    }
}

QT_END_NAMESPACE