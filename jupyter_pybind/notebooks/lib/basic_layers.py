
from bokeh.plotting import ColumnDataSource
from bokeh.models import  LabelSet,TableColumn,DataTable

class PointsLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
    })

    self.plot = self.fig.scatter(self.xs,
                  self.ys,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
    })

class PointsLayerWithColors:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys, self.color = 'pts_xs', 'pts_ys', 'color'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.color: []
    })

    self.plot = self.fig.scatter(self.xs,
                  self.ys,
                  color = self.color, 
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys, color):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
      self.color: color
    })

class TrianglePointsLayer:
  def __init__(self, fig, type, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.type = type
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
    })

    if self.type == 'triangle':
      self.plot = self.fig.triangle(self.xs,
                    self.ys,
                    source=self.data_source,
                    **params)
    
    elif self.type == 'inverted_triangle':
      self.plot = self.fig.inverted_triangle(self.xs,
                    self.ys,
                    source=self.data_source,
                    **params)

  def update(self, pts_xs, pts_ys):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
    })

class CircleLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys, self.rs = 'pts_xs', 'pts_ys', 'rs'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.rs: []
    })

    self.plot = self.fig.circle(self.xs,
                  self.ys,
                  radius = self.rs,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys, pts_rs):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
      self.rs: pts_rs
    })
    
class DotLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: []
    })

    self.plot = self.fig.circle(self.xs,
                  self.ys,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys
    })

class MultiPolygonColorLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys, self.color = 'pts_xs', 'pts_ys', 'color'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.color: []
    })

    self.plot = self.fig.multi_polygons(self.xs,
                  self.ys,
                  color = self.color, 
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys, color):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
      self.color: color
    })

class CurveLayer:
    # 
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
    })

    self.plot = self.fig.line(self.xs,
                  self.ys,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
    })
    
class MultiCurveLayer:
    # 
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
    })

    self.plot = self.fig.multi_line(self.xs,
                  self.ys,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
    })
    
    
class CurveLayerV2:
    # 
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys, self.type = 'pts_xs', 'pts_ys', 'type'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      # self.type :[]
    })

    self.plot = self.fig.line(self.xs,
                  self.ys,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys, type):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
      # self.type: type
    })

class MultiLinesLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
    })

    self.plot = self.fig.multi_line(self.xs,
                  self.ys,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
    })

class MultiPolygonLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
    })

    self.plot = self.fig.multi_polygons(self.xs,
                  self.ys,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
    })

class MultiArcsLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.rs, self.min_angle, self.max_angle = 'rs', 'min_angle', 'max_angle'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.rs: [],
      self.min_angle: [],
      self.max_angle: []
    })

    self.plot = self.fig.arc(
      self.xs,self.ys,
      radius = self.rs,
      start_angle = self.min_angle,
      end_angle = self.max_angle,
      source=self.data_source,
      **params)
  def update(self, pts_xs, pts_ys, rs, min_angle, max_angle):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
      self.rs: rs,
      self.min_angle: min_angle,
      self.max_angle: max_angle
    })
class MultiWedgesLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.rs, self.min_angle, self.max_angle = 'rs', 'min_angle', 'max_angle'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.rs: [],
      self.min_angle: [],
      self.max_angle: []
    })

    self.plot = self.fig.wedge(
      self.xs,self.ys,
      radius = self.rs,
      start_angle = self.min_angle,
      end_angle = self.max_angle,
      source=self.data_source,
      **params)

  def update(self, pts_xs, pts_ys, rs, min_angle, max_angle):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
      self.rs: rs,
      self.min_angle: min_angle,
      self.max_angle: max_angle
    })

class TextLabelLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.texts = 'texts'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.texts: []
    })
    
    self.plot = LabelSet(x=self.xs, y=self.ys, text=self.texts,
          x_offset=0, y_offset=0, source=self.data_source, render_mode='canvas', **params)
    
    # print(self.plot.__dict__)
    fig.add_layout(self.plot)
    
  def update(self, pts_xs, pts_ys, texts):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
      self.texts: texts
    })

class TableLayer:
  def __init__(self, fig, tab_attr_list, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.texts = 'texts'
    self.data_source = ColumnDataSource(data={
        self.xs: [],
        self.ys: [],
        self.texts: []
        })
    self.tableColumns = [
        TableColumn(field="pts_xs",title=tab_attr_list[0]),
        TableColumn(field="pts_ys",title=tab_attr_list[1]),
        TableColumn(field="texts",title=tab_attr_list[2])
    ]

    self.plot = DataTable(source=self.data_source, columns=self.tableColumns, **params)
    
  def update(self, pts_xs, pts_ys, texts):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
      self.texts: texts
    })
class TableLayerV2:
  def __init__(self, fig, tab_attr_list, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.texts = 'texts'
    self.data_source = ColumnDataSource(data={
        self.xs: [],
        self.ys: [],
        })
    self.tableColumns = [
        TableColumn(field="pts_xs",title=tab_attr_list[0]),
        TableColumn(field="pts_ys",title=tab_attr_list[1])
    ]

    self.plot = DataTable(source=self.data_source, columns=self.tableColumns, **params)
    
  def update(self, pts_xs, pts_ys, texts):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys
    })
    
class PatchLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys = 'pts_xs', 'pts_ys'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
    })

    self.plot = self.fig.patches(self.xs,
                  self.ys,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
    }) 
    
class TextLayer:
  def __init__(self, fig, params):
    self.fig = fig
    self.xs, self.ys, self.text = 'pts_xs', 'pts_ys', 'texts'
    self.data_source = ColumnDataSource(data={
      self.xs: [],
      self.ys: [],
      self.text :[],
    })

    self.plot = self.fig.text(self.xs,
                  self.ys,
                  self.text,
                  source=self.data_source,
                  **params)

  def update(self, pts_xs, pts_ys, texts):
    self.data_source.data.update({
      self.xs: pts_xs,
      self.ys: pts_ys,
      self.text: texts
    })