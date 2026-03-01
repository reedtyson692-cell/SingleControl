#pragma once

#include <QtCharts/QChart>
QT_CHARTS_BEGIN_NAMESPACE
class QSplineSeries;
class QValueAxis;
QT_CHARTS_END_NAMESPACE

QT_CHARTS_USE_NAMESPACE    //必须包含

class MyChart : public QChart
{
	Q_OBJECT

public:
	//explicit QChart(QGraphicsItem* parent = nullptr, Qt::WindowFlags wFlags = Qt::WindowFlags());
	MyChart(QGraphicsItem* parent = 0, Qt::WindowFlags wFlags = 0);
	~MyChart();

public:
	//定义线型
	QSplineSeries* m_pSeries;
	QSplineSeries* m_pSeries1;
	//定义x轴
	QValueAxis* m_pAxisx;
	//定义y轴
	QValueAxis* m_pAxisy;

	//代表曲线的起始坐标
	qreal m_x;
	qreal m_y1;
	qreal m_y2;
};
