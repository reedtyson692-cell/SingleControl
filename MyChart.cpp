#include "MyChart.h"
#include <QtCharts/QValueAxis>
#include <QtCharts/QSplineSeries>

MyChart::MyChart(QGraphicsItem* parent, Qt::WindowFlags wFlags)
: QChart(parent, wFlags)
{
	m_pAxisx = new QValueAxis();
	m_pAxisy = new QValueAxis();

	m_x = 0;
	m_y1 = 0;
	m_y2 = 0;

	QPen pen(Qt::red);
	QPen pen1(Qt::yellow);

	pen.setWidth(2);
	pen1.setWidth(2);

	m_pSeries = new QSplineSeries(this);
	m_pSeries->setPen(pen);
	m_pSeries->append(m_x, m_y1);

	m_pSeries1 = new QSplineSeries(this);
	m_pSeries1->setPen(pen1);
	m_pSeries1->append(m_x, m_y2);

	addSeries(m_pSeries);
	addSeries(m_pSeries1);

	//设置坐标轴的位置
	addAxis(m_pAxisx, Qt::AlignBottom);
	addAxis(m_pAxisy, Qt::AlignLeft);

	m_pSeries->attachAxis(m_pAxisx);
	m_pSeries->attachAxis(m_pAxisy);

	m_pSeries1->attachAxis(m_pAxisx);
	m_pSeries1->attachAxis(m_pAxisy);

	//设置刻度数
	m_pAxisx->setTickCount(25);
	m_pAxisy->setTickCount(10);

	//设置刻度范围
	m_pAxisx->setRange(0, 50);
	m_pAxisy->setRange(110, 150);

}

MyChart::~MyChart()
{

}
