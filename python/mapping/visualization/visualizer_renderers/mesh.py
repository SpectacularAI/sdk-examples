import pathlib
import numpy as np
from OpenGL.GL import *
from .util import *

COORDS_PER_VERTEX = 3
COLORS_PER_VERTEX = 3

class Mesh:
    def __init__(self, vertices, colors, indices):
        # 1d arrays of GLfloat.
        self.vertices = vertices
        self.colors = colors
        self.indices = indices

    def size(self):
        return self.vertices.shape[0] // COORDS_PER_VERTEX

class MeshProgram:
    def __init__(self, program):
        assert(program)
        self.program = program
        self.uniformModelView = glGetUniformLocation(self.program, "u_ModelView")
        self.uniformProjection = glGetUniformLocation(self.program, "u_Projection")
        self.uniformOpacity = glGetUniformLocation(self.program, "u_Opacity")

class MeshRenderer:
    meshProgram = None
    mesh = None
    updated = False
    vbo_position = None
    vbo_color = None
    ebo = None

    def __init__(self, mesh=None, opacity=1.0):
        self.mesh = mesh
        self.updated = mesh is not None
        self.opacity = opacity

    def render(self, modelMatrix, viewMatrix, projectionMatrix):
        if self.meshProgram is None:
            assetDir = pathlib.Path(__file__).resolve().parent
            vert = (assetDir / "mesh.vert").read_text()
            frag = (assetDir / "mesh.frag").read_text()
            self.meshProgram = MeshProgram(createProgram(vert, frag))

        if self.mesh is None: return
        modelView = viewMatrix @ modelMatrix

        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glEnable(GL_DEPTH_TEST)
        if self.opacity >= 1.0: glDepthMask(GL_TRUE)
        else: glDepthMask(GL_FALSE)

        glUseProgram(self.meshProgram.program)
        glUniformMatrix4fv(self.meshProgram.uniformModelView, 1, GL_FALSE, modelView.transpose())
        glUniformMatrix4fv(self.meshProgram.uniformProjection, 1, GL_FALSE, projectionMatrix.transpose())
        glUniform1f(self.meshProgram.uniformOpacity, self.opacity)

        if self.updated:
            self.updated = False

            # Delete old VBOs
            if self.vbo_position is not None:
                glDeleteBuffers(1, [self.vbo_position, self.vbo_color, self.ebo])

            # Create new VBOs
            self.vbo_position = glGenBuffers(1)
            self.vbo_color = glGenBuffers(1)
            self.ebo = glGenBuffers(1)

            # Bind and populate the new VBOs with updated data
            glBindBuffer(GL_ARRAY_BUFFER, self.vbo_position)
            glBufferData(GL_ARRAY_BUFFER, np.array(self.mesh.vertices, dtype=np.float32), GL_STATIC_DRAW)

            glBindBuffer(GL_ARRAY_BUFFER, self.vbo_color)
            glBufferData(GL_ARRAY_BUFFER, np.array(self.mesh.colors, dtype=np.float32), GL_STATIC_DRAW)

            glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)
            glBufferData(GL_ELEMENT_ARRAY_BUFFER, self.mesh.indices, GL_STATIC_DRAW)

        # Draw mesh
        glEnableVertexAttribArray(0)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_position)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, None)

        glEnableVertexAttribArray(1)
        glBindBuffer(GL_ARRAY_BUFFER, self.vbo_color)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, None)

        # Bind and render using the EBO
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, self.ebo)
        glDrawElements(GL_TRIANGLES, len(self.mesh.indices), GL_UNSIGNED_INT, None)

        glDisableVertexAttribArray(0)
        glDisableVertexAttribArray(1)

        glUseProgram(0)
        glDisable(GL_BLEND)
        glDisable(GL_DEPTH_TEST)

    def setMesh(self, mesh):
        self.updated = True
        self.mesh = mesh

    def setOpacity(self, opacity):
        self.opacity = opacity

