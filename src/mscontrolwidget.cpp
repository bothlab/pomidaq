/*
 * Copyright (C) 2019-2026 Matthias Klumpp <matthias@tenstral.net>
 *
 * Licensed under the GNU Lesser General Public License Version 3
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the license, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this software.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "mscontrolwidget.h"

#include <QSlider>
#include <QSpinBox>
#include <QLabel>
#include <QHBoxLayout>

MSControlWidget::MSControlWidget(const Miniscope::ControlDefinition &ctlDef, QWidget *parent)
    : QWidget(parent)
{
    m_controlId = QString::fromStdString(ctlDef.id);

    // all controls are laid out in a single row: title | slider | value
    const auto layout = new QHBoxLayout(this);
    layout->setContentsMargins(2, 0, 2, 0);
    layout->setSpacing(6);

    auto lblTitle = new QLabel(QString::fromStdString(ctlDef.name), this);
    // give titles a common minimum width, so sliders of consecutive controls line up
    lblTitle->setMinimumWidth(fontMetrics().horizontalAdvance(QStringLiteral("Excitation")) + 4);
    layout->addWidget(lblTitle);

    m_slider = new QSlider(Qt::Horizontal, this);
    m_slider->setRange(ctlDef.valueMin, ctlDef.valueMax);
    m_slider->setValue(ctlDef.valueStart);
    layout->addWidget(m_slider, 1);

    if (ctlDef.kind == Miniscope::ControlKind::Selector) {
        QStringList labels;
        for (const auto &label : ctlDef.labels)
            labels.append(QString::fromStdString(label));

        m_slider->setSingleStep(1);
        m_slider->setPageStep(1);
        m_slider->setTickPosition(QSlider::TicksBelow);
        m_slider->setTickInterval(1);

        // label showing the name of the currently selected value
        auto lblValue = new QLabel(this);
        lblValue->setAlignment(Qt::AlignRight | Qt::AlignVCenter);

        // reserve space for the widest label, so the slider does not jump around when the value changes
        int maxLabelWidth = 0;
        for (const auto &label : labels)
            maxLabelWidth = std::max(maxLabelWidth, fontMetrics().horizontalAdvance(label));
        lblValue->setMinimumWidth(maxLabelWidth + 4);
        layout->addWidget(lblValue);

        const auto valueMin = ctlDef.valueMin;
        const auto updateValueLabel = [lblValue, labels, valueMin](int value) {
            const auto idx = value - valueMin;
            if (idx >= 0 && idx < labels.size())
                lblValue->setText(labels[idx]);
            else
                lblValue->setText(QString::number(value));
        };
        updateValueLabel(m_slider->value());
        connect(m_slider, &QSlider::valueChanged, this, updateValueLabel);
    } else {
        m_slider->setSingleStep(ctlDef.stepSize);

        auto sb = new QSpinBox(this);
        sb->setRange(ctlDef.valueMin, ctlDef.valueMax);
        sb->setValue(ctlDef.valueStart);
        sb->setSingleStep(ctlDef.stepSize);
        sb->setMinimumWidth(64);
        layout->addWidget(sb);

        connect(sb, static_cast<void (QSpinBox::*)(int)>(&QSpinBox::valueChanged), m_slider, &QSlider::setValue);
        connect(m_slider, &QSlider::valueChanged, sb, &QSpinBox::setValue);
    }

    connect(m_slider, &QSlider::valueChanged, this, &MSControlWidget::recvSliderValueChange);
    setLayout(layout);
}

QString MSControlWidget::controlId() const
{
    return m_controlId;
}

double MSControlWidget::value() const
{
    return m_slider->value();
}

void MSControlWidget::setValue(double value)
{
    m_slider->setValue(value);
}

void MSControlWidget::recvSliderValueChange(int value)
{
    Q_EMIT valueChanged(m_controlId, value);
}
