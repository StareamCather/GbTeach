// Test-only bridge into the real Classic GUI. Never loaded by a normal lesson.
// Commands target observed QObject IDs in this client, not desktop coordinates.
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/gui/GuiIface.hh>
#include <gazebo/gui/MainWindow.hh>
#include <QAbstractButton>
#include <QAction>
#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QGraphicsView>
#include <QGraphicsItem>
#include <QGraphicsObject>
#include <QGroupBox>
#include <QGuiApplication>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLineEdit>
#include <QMetaMethod>
#include <QPointer>
#include <QSaveFile>
#include <QSet>
#include <QScreen>
#include <QSpinBox>
#include <QTest>
#include <QTabWidget>
#include <QTabBar>
#include <QTimer>
#include <QTreeWidget>
#include <functional>
#include <stdexcept>

namespace gazebo {
class EditorProbe : public GUIPlugin {
 public:
  EditorProbe() {
    this->setObjectName("GazeboTeachEditorProbe");
    this->setFixedSize(0, 0);
  }
  void Load(sdf::ElementPtr) override {
    directory = QString::fromLocal8Bit(qgetenv("GAZEBOTEACH_EDITOR_IO"));
    if (directory.isEmpty() || !QDir(directory).isAbsolute()) return;
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, [this]() { Poll(); });
    timer->start(150);
  }

 private:
  QString directory;
  QTimer *timer = nullptr;
  int lastSequence = -1;
  QHash<QString, QPointer<QObject>> objects;

  QString Id(QObject *object) {
    const QString key = QString::number(reinterpret_cast<quintptr>(object), 16);
    objects.insert(key, object);
    return key;
  }
  static QJsonArray Rect(const QRectF &r) {
    return QJsonArray{r.x(), r.y(), r.width(), r.height()};
  }
  QJsonObject Describe(QObject *object) {
    QJsonObject row{{"id", Id(object)}, {"class", object->metaObject()->className()},
                    {"name", object->objectName()}};
    const auto textProperty = object->property("text");
    if (textProperty.isValid()) row.insert("text", textProperty.toString());
    if (object->parent()) row.insert("parent", Id(object->parent()));
    if (auto widget = qobject_cast<QWidget *>(object)) {
      row.insert("visible", widget->isVisible());
      row.insert("enabled", widget->isEnabled());
      row.insert("rect", Rect(widget->rect()));
      row.insert("geometry", Rect(widget->geometry()));
      row.insert("title", widget->windowTitle());
    }
    if (auto action = qobject_cast<QAction *>(object)) {
      row.insert("text", action->text());
      row.insert("enabled", action->isEnabled());
      row.insert("checkable", action->isCheckable());
      row.insert("checked", action->isChecked());
      row.insert("shortcut", action->shortcut().toString());
    }
    if (auto button = qobject_cast<QAbstractButton *>(object)) {
      row.insert("text", button->text());
      row.insert("checked", button->isChecked());
    }
    if (auto line = qobject_cast<QLineEdit *>(object)) row.insert("text", line->text());
    if (auto spin = qobject_cast<QDoubleSpinBox *>(object)) row.insert("value", spin->value());
    if (auto spin = qobject_cast<QSpinBox *>(object)) row.insert("value", spin->value());
    if (auto group = qobject_cast<QGroupBox *>(object)) row.insert("group_title", group->title());
    if (auto tabs = qobject_cast<QTabWidget *>(object)) {
      QJsonArray names;
      for (int i = 0; i < tabs->count(); ++i) names.append(tabs->tabText(i));
      row.insert("tabs", names); row.insert("current_index", tabs->currentIndex());
    }
    if (auto tree = qobject_cast<QTreeWidget *>(object)) {
      QJsonArray items;
      std::function<void(QTreeWidgetItem *, QJsonArray)> visit;
      visit = [&](QTreeWidgetItem *item, QJsonArray path) {
        items.append(QJsonObject{{"path", path}, {"text", item->text(0)},
                                 {"rect", Rect(tree->visualItemRect(item))}});
        for (int i = 0; i < item->childCount(); ++i) {
          auto childPath = path; childPath.append(i); visit(item->child(i), childPath);
        }
      };
      for (int i = 0; i < tree->topLevelItemCount(); ++i) visit(tree->topLevelItem(i), QJsonArray{i});
      row.insert("tree_items", items);
    }
    if (auto combo = qobject_cast<QComboBox *>(object)) {
      QJsonArray items;
      for (int i = 0; i < combo->count(); ++i) items.append(combo->itemText(i));
      row.insert("items", items);
      row.insert("text", combo->currentText());
    }
    if (auto view = qobject_cast<QGraphicsView *>(object)) {
      row.insert("viewport", Id(view->viewport()));
      row.insert("scene_rect", Rect(view->sceneRect()));
      QJsonArray items;
      if (view->scene()) for (auto item : view->scene()->items()) {
        if (items.size() >= 150) break;
        QJsonObject entry{{"type", item->type()}, {"visible", item->isVisible()},
                          {"scene_bounds", Rect(item->sceneBoundingRect())}};
        if (auto obj = item->toGraphicsObject()) entry.insert("class", obj->metaObject()->className());
        items.append(entry);
      }
      row.insert("scene_items", items);
    }
    QJsonArray methods;
    const auto meta = object->metaObject();
    for (int i = meta->methodOffset(); i < meta->methodCount(); ++i)
      methods.append(QString::fromLatin1(meta->method(i).methodSignature()));
    row.insert("methods", methods);
    return row;
  }
  QJsonArray Snapshot() {
    QJsonArray rows;
    QSet<QObject *> seen;
    auto add = [&](QObject *object) {
      if (object && !seen.contains(object)) { seen.insert(object); rows.append(Describe(object)); }
    };
    for (auto widget : QApplication::allWidgets()) add(widget);
    if (auto window = gui::get_main_window()) {
      add(window);
      for (auto object : window->findChildren<QObject *>()) add(object);
    }
    return rows;
  }
  QObject *Target(const QJsonObject &command) {
    const auto target = objects.value(command.value("id").toString());
    if (!target) throw std::runtime_error("Observed QObject no longer exists; take a fresh snapshot");
    return target;
  }
  QJsonObject Execute(const QJsonObject &command) {
    const QString verb = command.value("verb").toString();
    if (verb == "snapshot") return QJsonObject{{"objects", Snapshot()}};
    QObject *target = Target(command);
    if (verb == "trigger") {
      auto action = qobject_cast<QAction *>(target);
      if (!action || !action->isEnabled()) throw std::runtime_error("Target is not an enabled QAction");
      // Save opens a nested modal loop. Acknowledge before entering that loop
      // so the controller can inspect and fill the genuine Gazebo dialog.
      QPointer<QAction> guarded(action);
      QTimer::singleShot(0, this, [guarded]() { if (guarded) guarded->trigger(); });
    } else if (verb == "click") {
      auto button = qobject_cast<QAbstractButton *>(target);
      if (!button || !button->isVisible() || !button->isEnabled())
        throw std::runtime_error("Target is not a visible enabled button");
      QPointer<QAbstractButton> guarded(button);
      QTimer::singleShot(0, this, [guarded]() { if (guarded) guarded->click(); });
    } else if (verb == "text") {
      auto line = qobject_cast<QLineEdit *>(target);
      if (!line) throw std::runtime_error("Target is not a QLineEdit");
      line->setText(command.value("text").toString());
      QMetaObject::invokeMethod(line, "editingFinished", Qt::DirectConnection);
    } else if (verb == "value") {
      const double value = command.value("value").toDouble();
      if (auto spin = qobject_cast<QDoubleSpinBox *>(target)) spin->setValue(value);
      else if (auto spin = qobject_cast<QSpinBox *>(target)) spin->setValue(static_cast<int>(value));
      else throw std::runtime_error("Target is not a spin box");
    } else if (verb == "tab") {
      auto tabs = qobject_cast<QTabWidget *>(target);
      const int index = command.value("index").toInt(-1);
      if (!tabs || index < 0 || index >= tabs->count()) throw std::runtime_error("Invalid observed tab");
      tabs->setCurrentIndex(index);
    } else if (verb == "tree_double") {
      auto tree = qobject_cast<QTreeWidget *>(target);
      const auto path = command.value("path").toArray();
      if (!tree || path.isEmpty()) throw std::runtime_error("Invalid observed tree path");
      QTreeWidgetItem *item = tree->topLevelItem(path[0].toInt(-1));
      for (int i = 1; item && i < path.size(); ++i) { tree->expandItem(item); item = item->child(path[i].toInt(-1)); }
      if (!item) throw std::runtime_error("Observed tree item no longer exists");
      tree->scrollToItem(item); tree->setCurrentItem(item);
      const auto point = tree->visualItemRect(item).center();
      QTest::mouseClick(tree->viewport(), Qt::LeftButton, Qt::NoModifier, point, 20);
      QTest::mouseDClick(tree->viewport(), Qt::LeftButton, Qt::NoModifier, point, 20);
    } else if (verb == "checked") {
      auto button = qobject_cast<QAbstractButton *>(target);
      if (!button || !button->isCheckable()) throw std::runtime_error("Target is not checkable");
      button->setChecked(command.value("value").toBool());
    } else if (verb == "mouse_move" || verb == "mouse_click" || verb == "mouse_double") {
      auto widget = qobject_cast<QWidget *>(target);
      if (!widget || !widget->isVisible()) throw std::runtime_error("Mouse target is not visible");
      const QPoint position(command.value("x").toInt(), command.value("y").toInt());
      if (!widget->rect().contains(position)) throw std::runtime_error("Mouse point outside observed target");
      const auto modifiers = static_cast<Qt::KeyboardModifiers>(command.value("modifiers").toInt());
      if (verb == "mouse_move") QTest::mouseMove(widget, position, 20);
      else if (verb == "mouse_double") QTest::mouseDClick(widget, Qt::LeftButton, modifiers, position, 20);
      else QTest::mouseClick(widget, Qt::LeftButton, modifiers, position, 20);
    } else if (verb == "key") {
      auto widget = qobject_cast<QWidget *>(target);
      if (!widget) throw std::runtime_error("Key target is not a widget");
      const auto key = static_cast<Qt::Key>(command.value("key").toInt());
      const auto modifiers = static_cast<Qt::KeyboardModifiers>(command.value("modifiers").toInt());
      QPointer<QWidget> guarded(widget);
      QTimer::singleShot(0, this, [guarded, key, modifiers]() {
        if (guarded) QTest::keyClick(guarded, key, modifiers, 20);
      });
    } else if (verb == "screenshot") {
      auto widget = qobject_cast<QWidget *>(target);
      if (!widget || !QGuiApplication::primaryScreen()->grabWindow(widget->winId()).save(command.value("path").toString()))
        throw std::runtime_error("Cannot capture the owned QWidget");
    } else throw std::runtime_error("Unknown test command");
    return QJsonObject{{"scheduled", true}};
  }
  void Poll() {
    QFile file(directory + "/command.json");
    if (!file.open(QIODevice::ReadOnly)) return;
    const auto document = QJsonDocument::fromJson(file.readAll());
    if (!document.isObject()) return;
    const auto command = document.object();
    const int sequence = command.value("seq").toInt(-1);
    if (sequence < 0 || sequence == lastSequence) return;
    lastSequence = sequence;
    QJsonObject response{{"seq", sequence}, {"ok", true}};
    try { response.insert("data", Execute(command)); }
    catch (const std::exception &error) {
      response.insert("ok", false); response.insert("error", error.what());
    }
    QSaveFile output(directory + "/response.json");
    if (output.open(QIODevice::WriteOnly)) {
      output.write(QJsonDocument(response).toJson()); output.commit();
    }
  }
};
GZ_REGISTER_GUI_PLUGIN(EditorProbe)
}  // namespace gazebo
