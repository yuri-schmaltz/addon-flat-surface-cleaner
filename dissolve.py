bl_info = {
    "name": "Flat Surface Cleaner",
    "author": "ChatGPT",
    "version": (1, 1, 0),
    "blender": (3, 6, 0),
    "location": "View3D > Sidebar (N) > Mesh > Flat Surface Cleaner",
    "description": "Torna a seleção totalmente plana e reconstrói como uma única face (sem geometria interna extra).",
    "category": "Mesh",
}

import bpy
import bmesh
import math
from mathutils import Vector, Matrix


# ============================================================
# Localização simples (PT/EN)
# ============================================================
DEFAULT_LANGUAGE = "PT"

LOCALE_STRINGS = {
    "PT": {
        "prefs_language": "Idioma",
        "prefs_language_desc": "Escolha o idioma das labels e mensagens do addon.",
        "prefs_language_label": "Idioma do Add-on",
        "prefs_language_prop": "Idioma",
        "plane_mode": "Plano de Referência",
        "plane_mode_desc": "Como definir o plano final",
        "plane_best_fit": "Melhor Ajuste",
        "plane_best_fit_desc": "Plano de melhor ajuste pelos vértices selecionados",
        "plane_active": "Face Ativa",
        "plane_active_desc": "Usa a normal/centro da face ativa (deve estar na seleção)",
        "plane_average": "Média das Normais",
        "plane_average_desc": "Média ponderada das normais das faces selecionadas",
        "remove_doubles": "Weld no Contorno",
        "remove_doubles_desc": "Mescla pontos muito próximos no contorno antes de criar a face",
        "merge_distance": "Distância Weld",
        "simplify_boundary": "Simplificar Contorno",
        "simplify_boundary_desc": "Dissolve vértices colineares no contorno (reduz pontos 'extras' no perímetro)",
        "simplify_angle": "Tolerância (°)",
        "simplify_angle_desc": "Quanto mais alto, mais agressivo ao remover vértices colineares",
        "keep_largest_loop": "Usar Apenas o Maior Contorno",
        "keep_largest_loop_desc": "Se houver múltiplos contornos, mantém apenas o de maior área (preenche 'furos')",
        "recalc_normals": "Recalcular Normais",
        "panel_label": "Flat Surface Cleaner",
        "section_plane": "Plano / Reconstrução:",
        "section_contour": "Contorno:",
        "operator_label": "Planarizar e Recriar como 1 Face",
        "report_select_faces": "Selecione FACES (uma região de faces) antes de executar.",
        "report_minimum_selection": "Seleção insuficiente (mínimo 3 vértices).",
        "report_no_boundary": "Não foi encontrado contorno. A seleção parece não definir uma 'tampa' aberta.",
        "report_invalid_active": "Face ativa inválida. Ative uma face dentro da seleção ou use 'Melhor Ajuste'.",
        "report_invalid_selection": "A seleção ficou inválida após weld (sem faces).",
        "report_non_manifold_boundary": "Contorno inválido: há vértice com menos de duas arestas ou com ramificações.",
        "report_invalid_loop": "Contorno inválido (não foi possível formar loop fechado).",
        "report_invalid_loop_after_cleanup": "Loop inválido após limpeza (contorno insuficiente).",
        "report_create_face_fail": "Falha ao criar uma única face. Contorno pode estar auto-intersectando ou não-manifold.",
    },
    "EN": {
        "prefs_language": "Language",
        "prefs_language_desc": "Choose the language for the add-on labels and messages.",
        "prefs_language_label": "Add-on Language",
        "prefs_language_prop": "Language",
        "plane_mode": "Reference Plane",
        "plane_mode_desc": "How to define the final plane",
        "plane_best_fit": "Best Fit",
        "plane_best_fit_desc": "Best-fit plane using the selected vertices",
        "plane_active": "Active Face",
        "plane_active_desc": "Use the active face normal/center (must be within the selection)",
        "plane_average": "Average Normals",
        "plane_average_desc": "Weighted average of the selected faces' normals",
        "remove_doubles": "Boundary Weld",
        "remove_doubles_desc": "Merge very close points on the boundary before creating the face",
        "merge_distance": "Weld Distance",
        "simplify_boundary": "Simplify Boundary",
        "simplify_boundary_desc": "Dissolve collinear boundary vertices (removes extra perimeter points)",
        "simplify_angle": "Tolerance (°)",
        "simplify_angle_desc": "Higher values remove collinear vertices more aggressively",
        "keep_largest_loop": "Use Only Largest Boundary",
        "keep_largest_loop_desc": "If multiple boundaries exist, keep only the one with the largest area (fills holes)",
        "recalc_normals": "Recalculate Normals",
        "panel_label": "Flat Surface Cleaner",
        "section_plane": "Plane / Rebuild:",
        "section_contour": "Boundary:",
        "operator_label": "Flatten and Rebuild as 1 Face",
        "report_select_faces": "Select FACES (a face region) before running.",
        "report_minimum_selection": "Selection too small (minimum 3 vertices).",
        "report_no_boundary": "No boundary found. The selection does not seem to define an open cap.",
        "report_invalid_active": "Invalid active face. Activate a face inside the selection or use 'Best Fit'.",
        "report_invalid_selection": "Selection became invalid after weld (no faces).",
        "report_non_manifold_boundary": "Invalid boundary: a vertex has fewer than two edges or branches.",
        "report_invalid_loop": "Invalid boundary (could not form a closed loop).",
        "report_invalid_loop_after_cleanup": "Invalid loop after cleanup (insufficient boundary).",
        "report_create_face_fail": "Failed to create a single face. Boundary may self-intersect or be non-manifold.",
    },
}


def _get_language():
    try:
        prefs = bpy.context.preferences
        if prefs:
            addon = prefs.addons.get(__name__)
            if addon and hasattr(addon, "preferences"):
                return addon.preferences.language
    except Exception:
        pass
    return DEFAULT_LANGUAGE


def L(key: str) -> str:
    lang = _get_language()
    locale = LOCALE_STRINGS.get(lang) or LOCALE_STRINGS.get(DEFAULT_LANGUAGE, {})
    fallback = LOCALE_STRINGS.get(DEFAULT_LANGUAGE, {})
    return locale.get(key) or fallback.get(key, key)


# ============================================================
# Matemática: plano de melhor ajuste (sem numpy)
# ============================================================
def _deg_to_rad(d: float) -> float:
    return d * math.pi / 180.0


def _solve_3x3(A: Matrix, b: Vector) -> Vector:
    """Resolve A x = b por eliminação de Gauss (A 3x3)."""
    m = [[A[0][0], A[0][1], A[0][2], b[0]],
         [A[1][0], A[1][1], A[1][2], b[1]],
         [A[2][0], A[2][1], A[2][2], b[2]]]

    # pivoteamento parcial
    for i in range(3):
        pivot = i
        maxv = abs(m[i][i])
        for r in range(i + 1, 3):
            v = abs(m[r][i])
            if v > maxv:
                maxv = v
                pivot = r
        if maxv < 1e-14:
            raise ZeroDivisionError("Matriz singular/quase singular.")
        if pivot != i:
            m[i], m[pivot] = m[pivot], m[i]

        # elimina abaixo
        piv = m[i][i]
        for r in range(i + 1, 3):
            f = m[r][i] / piv
            m[r][i] = 0.0
            m[r][1] -= f * m[i][1]
            m[r][2] -= f * m[i][2]
            m[r][3] -= f * m[i][3]

    # retro-substituição
    x = [0.0, 0.0, 0.0]
    for i in (2, 1, 0):
        s = m[i][3]
        for j in range(i + 1, 3):
            s -= m[i][j] * x[j]
        x[i] = s / m[i][i]
    return Vector((x[0], x[1], x[2]))


def _best_fit_plane(verts):
    """Retorna (normal, ponto_no_plano) via covariância + inverse iteration."""
    pts = [v.co.copy() for v in verts]
    if len(pts) < 3:
        return Vector((0.0, 0.0, 1.0)), pts[0] if pts else Vector((0.0, 0.0, 0.0))

    c = Vector((0.0, 0.0, 0.0))
    for p in pts:
        c += p
    c /= len(pts)

    xx = xy = xz = yy = yz = zz = 0.0
    for p in pts:
        r = p - c
        xx += r.x * r.x
        xy += r.x * r.y
        xz += r.x * r.z
        yy += r.y * r.y
        yz += r.y * r.z
        zz += r.z * r.z

    C = Matrix(((xx, xy, xz),
                (xy, yy, yz),
                (xz, yz, zz)))

    eps = 1e-12 * (xx + yy + zz + 1.0)
    A = C + Matrix.Identity(3) * eps

    x = Vector((1.0, 0.3, 0.2)).normalized()
    for _ in range(24):
        try:
            y = _solve_3x3(A, x)
        except ZeroDivisionError:
            # fallback simples
            return Vector((0.0, 0.0, 1.0)), c
        if y.length < 1e-14:
            break
        x = y.normalized()

    n = x.normalized()
    if n.length < 1e-12:
        n = Vector((0.0, 0.0, 1.0))
    return n, c


def _average_face_normal(faces):
    n = Vector((0.0, 0.0, 0.0))
    for f in faces:
        try:
            n += f.normal * f.calc_area()
        except Exception:
            n += f.normal
    if n.length < 1e-12:
        return Vector((0.0, 0.0, 1.0))
    return n.normalized()


def _make_plane_basis(n: Vector):
    """Cria base ortonormal (u,v) no plano."""
    u = n.orthogonal()
    if u.length < 1e-12:
        u = Vector((1.0, 0.0, 0.0))
    u.normalize()
    v = n.cross(u)
    if v.length < 1e-12:
        v = Vector((0.0, 1.0, 0.0))
    v.normalize()
    return u, v


def _poly_area_2d(loop_verts, origin, u, v):
    """Área assinada (módulo) do polígono projetado no plano."""
    if len(loop_verts) < 3:
        return 0.0
    pts2 = []
    for bv in loop_verts:
        p = bv.co - origin
        pts2.append((p.dot(u), p.dot(v)))
    area = 0.0
    for i in range(len(pts2)):
        x1, y1 = pts2[i]
        x2, y2 = pts2[(i + 1) % len(pts2)]
        area += x1 * y2 - x2 * y1
    return abs(area) * 0.5


# ============================================================
# Topologia: boundary loop + rebuild em 1 face
# ============================================================
def _selected_faces(bm):
    return [f for f in bm.faces if f.select]


def _boundary_edges_of_selected_faces(sel_faces):
    sel_set = set(sel_faces)
    boundary = set()
    for f in sel_faces:
        for e in f.edges:
            count = 0
            for lf in e.link_faces:
                if lf in sel_set:
                    count += 1
            if count == 1:
                boundary.add(e)
    return boundary


def _edges_to_loops(edges):
    """Extrai loops (cada loop = lista ordenada de BMVert) de um conjunto de arestas de contorno."""
    edges = {e for e in edges if getattr(e, "is_valid", False)}
    if not edges:
        return []

    # adjacência
    adj = {}
    for e in edges:
        v1, v2 = e.verts[0], e.verts[1]
        adj.setdefault(v1, []).append(v2)
        adj.setdefault(v2, []).append(v1)

    unused = set(edges)
    loops = []

    def _find_edge(a, b):
        for ee in a.link_edges:
            if ee in edges and ee in unused:
                if (ee.verts[0] == b) or (ee.verts[1] == b):
                    return ee
        return None

    while unused:
        e0 = next(iter(unused))
        v_start = e0.verts[0]
        v_next = e0.verts[1]
        loop = [v_start, v_next]
        unused.discard(e0)

        prev = v_start
        curr = v_next

        # segue até fechar ou travar
        guard = 0
        while guard < 200000:
            guard += 1
            neigh = adj.get(curr, [])
            if len(neigh) < 1:
                break
            # tenta escolher o próximo diferente do anterior
            if len(neigh) == 1:
                cand = neigh[0]
            else:
                cand = neigh[0] if neigh[0] != prev else neigh[1]

            if cand == v_start:
                # fechou
                break

            ee = _find_edge(curr, cand)
            if ee is None:
                # pode ser que a aresta exista mas já foi consumida; tenta outro vizinho
                if len(neigh) > 2:
                    found = False
                    for alt in neigh:
                        if alt == prev or alt == v_start:
                            continue
                        ee2 = _find_edge(curr, alt)
                        if ee2:
                            cand = alt
                            ee = ee2
                            found = True
                            break
                    if not found:
                        break
                else:
                    break

            loop.append(cand)
            unused.discard(ee)
            prev, curr = curr, cand

        # valida loop fechado (último conecta no primeiro)
        if len(loop) >= 3 and (loop[-1] in adj.get(v_start, [])):
            loops.append(loop)

    return loops


def _boundary_is_manifold(edges):
    """Retorna True se cada vértice de contorno se conecta a exatamente duas arestas."""
    counts = {}
    for e in edges:
        for v in e.verts:
            counts[v] = counts.get(v, 0) + 1
    if not counts:
        return False
    return all(c == 2 for c in counts.values())


def _project_verts_to_plane(verts, origin: Vector, normal: Vector):
    n = normal.normalized()
    for v in verts:
        try:
            if not v.is_valid:
                continue
        except ReferenceError:
            continue
        d = (v.co - origin).dot(n)
        v.co -= n * d


def _dissolve_collinear_boundary(bm, loop_verts, angle_tol_rad: float):
    """Remove vértices colineares no contorno (reduz vertices 'inúteis' sem alterar forma)."""
    if angle_tol_rad <= 0.0:
        return
    if len(loop_verts) < 4:
        return

    to_dissolve = []
    n = len(loop_verts)
    for i in range(n):
        v_prev = loop_verts[(i - 1) % n]
        v = loop_verts[i]
        v_next = loop_verts[(i + 1) % n]
        if not (v_prev.is_valid and v.is_valid and v_next.is_valid):
            continue

        a = (v_prev.co - v.co)
        b = (v_next.co - v.co)
        if a.length < 1e-12 or b.length < 1e-12:
            continue
        ang = a.angle(b)
        # colinear se ~ 180 graus
        if abs(math.pi - ang) <= angle_tol_rad:
            to_dissolve.append(v)

    if to_dissolve:
        try:
            bmesh.ops.dissolve_verts(bm, verts=to_dissolve)
        except Exception:
            pass


# ============================================================
# Itens dinâmicos
# ============================================================
def _plane_mode_items(self, _context):
    return [
        ("BEST_FIT", L("plane_best_fit"), L("plane_best_fit_desc")),
        ("ACTIVE", L("plane_active"), L("plane_active_desc")),
        ("AVERAGE", L("plane_average"), L("plane_average_desc")),
    ]


# ============================================================
# Propriedades / UI
# ============================================================
class FSC_Settings(bpy.types.PropertyGroup):
    plane_mode: bpy.props.EnumProperty(
        name=L("plane_mode"),
        description=L("plane_mode_desc"),
        items=_plane_mode_items,
        default="BEST_FIT",
    )

    remove_doubles: bpy.props.BoolProperty(
        name=L("remove_doubles"),
        description=L("remove_doubles_desc"),
        default=True,
    )

    merge_distance: bpy.props.FloatProperty(
        name=L("merge_distance"),
        default=0.0001,
        min=0.0,
        max=0.1,
        precision=6,
    )

    simplify_boundary: bpy.props.BoolProperty(
        name=L("simplify_boundary"),
        description=L("simplify_boundary_desc"),
        default=False,
    )

    simplify_angle: bpy.props.FloatProperty(
        name=L("simplify_angle"),
        description=L("simplify_angle_desc"),
        default=0.2,
        min=0.0,
        max=5.0,
    )

    keep_largest_loop: bpy.props.BoolProperty(
        name=L("keep_largest_loop"),
        description=L("keep_largest_loop_desc"),
        default=True,
    )

    recalc_normals: bpy.props.BoolProperty(
        name=L("recalc_normals"),
        default=True,
    )


class FSC_AddonPreferences(bpy.types.AddonPreferences):
    bl_idname = __name__

    language: bpy.props.EnumProperty(
        name=L("prefs_language"),
        description=L("prefs_language_desc"),
        items=[
            ("PT", "Português", "Mostrar labels e mensagens em português"),
            ("EN", "English", "Show labels and messages in English"),
        ],
        default=DEFAULT_LANGUAGE,
    )

    def draw(self, context):
        layout = self.layout
        layout.label(text=L("prefs_language_label"))
        layout.prop(self, "language", text=L("prefs_language_prop"))


# ============================================================
# Operador principal: 1 seleção -> 1 face plana (sem internas)
# ============================================================
class FSC_OT_make_planar_single_face(bpy.types.Operator):
    bl_idname = "mesh.fsc_make_planar_single_face"
    bl_label = L("operator_label")
    bl_options = {"REGISTER", "UNDO"}

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob and ob.type == "MESH" and context.mode == "EDIT_MESH"

    def execute(self, context):
        st = context.scene.fsc_settings
        ob = context.active_object
        me = ob.data

        bm = bmesh.from_edit_mesh(me)
        bm.faces.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.verts.ensure_lookup_table()

        sel_faces = _selected_faces(bm)
        if not sel_faces:
            self.report({"WARNING"}, L("report_select_faces"))
            return {"CANCELLED"}

        sel_verts = {v for f in sel_faces for v in f.verts}
        if len(sel_verts) < 3:
            self.report({"WARNING"}, L("report_minimum_selection"))
            return {"CANCELLED"}

        boundary_edges = _boundary_edges_of_selected_faces(sel_faces)
        if not boundary_edges:
            # seleção sem contorno => superfície fechada/total; não dá para virar 'um tampo' só
            if len(sel_faces) == 1:
                # já é uma face: só planariza
                n, p0 = _best_fit_plane(sel_verts)
                _project_verts_to_plane(sel_verts, p0, n)
                bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)
                return {"FINISHED"}
            self.report({"ERROR"}, L("report_no_boundary"))
            return {"CANCELLED"}

        # Define o plano final
        if st.plane_mode == "ACTIVE":
            af = bm.faces.active
            if af is None or not af.select:
                self.report({"WARNING"}, L("report_invalid_active"))
                return {"CANCELLED"}
            normal = af.normal.normalized()
            origin = af.calc_center_median()
        elif st.plane_mode == "AVERAGE":
            normal = _average_face_normal(sel_faces)
            origin = sum((v.co for v in sel_verts), Vector((0.0, 0.0, 0.0))) / len(sel_verts)
        else:
            normal, origin = _best_fit_plane(sel_verts)

        # Planariza TUDO na seleção (inclui contorno) de forma exata
        _project_verts_to_plane(sel_verts, origin, normal)

        # Opcional: weld (apenas para reduzir duplicados no contorno antes do rebuild)
        if st.remove_doubles and st.merge_distance > 0.0:
            try:
                bmesh.ops.remove_doubles(bm, verts=list(sel_verts), dist=st.merge_distance)
            except Exception:
                pass

        bm.faces.ensure_lookup_table()
        bm.edges.ensure_lookup_table()
        bm.verts.ensure_lookup_table()

        # Recalcula boundary após weld
        sel_faces = [f for f in bm.faces if f.select]
        if not sel_faces:
            self.report({"ERROR"}, L("report_invalid_selection"))
            return {"CANCELLED"}

        sel_verts = {v for f in sel_faces for v in f.verts}
        boundary_edges = _boundary_edges_of_selected_faces(sel_faces)
        if not _boundary_is_manifold(boundary_edges):
            self.report({"ERROR"}, L("report_non_manifold_boundary"))
            return {"CANCELLED"}
        loops = _edges_to_loops(boundary_edges)
        if not loops:
            self.report({"ERROR"}, L("report_invalid_loop"))
            return {"CANCELLED"}

        # Escolhe loop (maior área no plano) se solicitado
        if st.keep_largest_loop and len(loops) > 1:
            u, v = _make_plane_basis(normal)
            loops_sorted = sorted(loops, key=lambda lp: _poly_area_2d(lp, origin, u, v), reverse=True)
            loop = loops_sorted[0]
            # descarta geometria de loops menores (preenche furos)
            keep_verts = set(loop)
            keep_edges = set()
            loop_set = set(loop)
            for e in boundary_edges:
                a, b = e.verts[0], e.verts[1]
                if a in loop_set and b in loop_set:
                    keep_edges.add(e)
            # remove quaisquer arestas/verts do contorno não pertencentes ao loop principal
            trash_edges = [e for e in boundary_edges if e not in keep_edges and getattr(e, "is_valid", False)]
            if trash_edges:
                try:
                    bmesh.ops.delete(bm, geom=trash_edges, context='EDGES')
                except Exception:
                    pass
        else:
            loop = loops[0]

        # Simplifica contorno (opcional)
        if st.simplify_boundary and st.simplify_angle > 0.0:
            _dissolve_collinear_boundary(bm, loop, _deg_to_rad(st.simplify_angle))
            bm.edges.ensure_lookup_table()
            bm.verts.ensure_lookup_table()
            # re-extraí loop do contorno atual (para garantir consistência)
            boundary_edges = _boundary_edges_of_selected_faces([f for f in bm.faces if f.select])
            if not _boundary_is_manifold(boundary_edges):
                self.report({"ERROR"}, L("report_non_manifold_boundary"))
                return {"CANCELLED"}
            loops2 = _edges_to_loops(boundary_edges)
            if loops2:
                if st.keep_largest_loop and len(loops2) > 1:
                    u, v = _make_plane_basis(normal)
                    loops2 = sorted(loops2, key=lambda lp: _poly_area_2d(lp, origin, u, v), reverse=True)
                loop = loops2[0]

        # Remove todas as faces selecionadas (mantendo contorno)
        # (remoção manual evita apagar contorno por contexto errado)
        for f in [f for f in bm.faces if f.select]:
            try:
                bm.faces.remove(f)
            except Exception:
                pass

        bm.edges.ensure_lookup_table()
        bm.verts.ensure_lookup_table()

        # Remove toda geometria interna restante (tudo que estava na seleção e não é do contorno)
        loop_set = set([v for v in loop if getattr(v, "is_valid", False)])
        internal_verts = [v for v in sel_verts if getattr(v, "is_valid", False) and v not in loop_set]
        if internal_verts:
            try:
                bmesh.ops.delete(bm, geom=internal_verts, context='VERTS')
            except Exception:
                # fallback: remove manual
                for vv in internal_verts:
                    try:
                        bm.verts.remove(vv)
                    except Exception:
                        pass

        bm.edges.ensure_lookup_table()
        bm.verts.ensure_lookup_table()

        # Garante que o loop ainda é válido e fechado
        # Reobtém arestas do loop (pelo grafo atual)
        # (operação final deve usar o loop em ordem)
        loop = [v for v in loop if getattr(v, "is_valid", False)]
        if len(loop) < 3:
            self.report({"ERROR"}, L("report_invalid_loop_after_cleanup"))
            return {"CANCELLED"}

        # Cria UMA face (ngon) com o contorno
        new_face = None
        try:
            new_face = bm.faces.new(loop)
        except ValueError:
            # Já existe uma face com esse ciclo (ou loop repetido). Tenta achar face existente.
            for f in bm.faces:
                vs = set(f.verts)
                if len(vs) == len(loop) and vs == set(loop):
                    new_face = f
                    break
        except Exception:
            new_face = None

        if new_face is None:
            self.report({"ERROR"}, L("report_create_face_fail"))
            return {"CANCELLED"}

        # Seleciona apenas a face final
        for v in bm.verts:
            v.select = False
        for e in bm.edges:
            e.select = False
        for f in bm.faces:
            f.select = False
        new_face.select = True
        bm.faces.active = new_face

        if st.recalc_normals:
            try:
                bmesh.ops.recalc_face_normals(bm, faces=[new_face])
            except Exception:
                pass

        bm.normal_update()
        bmesh.update_edit_mesh(me, loop_triangles=False, destructive=True)
        return {"FINISHED"}


# ============================================================
# Painel
# ============================================================
class FSC_PT_panel(bpy.types.Panel):
    bl_label = L("panel_label")
    bl_idname = "FSC_PT_panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Mesh"

    @classmethod
    def poll(cls, context):
        ob = context.active_object
        return ob and ob.type == "MESH"

    def draw(self, context):
        layout = self.layout
        st = context.scene.fsc_settings

        col = layout.column(align=True)
        col.label(text=L("section_plane"))
        col.prop(st, "plane_mode", text=L("plane_mode"))
        col.prop(st, "keep_largest_loop", text=L("keep_largest_loop"))

        layout.separator()

        col = layout.column(align=True)
        col.label(text=L("section_contour"))
        col.prop(st, "remove_doubles", text=L("remove_doubles"))
        sub = col.column(align=True)
        sub.enabled = st.remove_doubles
        sub.prop(st, "merge_distance", text=L("merge_distance"))
        col.prop(st, "simplify_boundary", text=L("simplify_boundary"))
        sub = col.column(align=True)
        sub.enabled = st.simplify_boundary
        sub.prop(st, "simplify_angle", text=L("simplify_angle"))

        layout.prop(st, "recalc_normals", text=L("recalc_normals"))

        layout.separator()
        layout.operator("mesh.fsc_make_planar_single_face", icon="MESH_GRID", text=L("operator_label"))


# ============================================================
# Registro
# ============================================================
classes = (
    FSC_AddonPreferences,
    FSC_Settings,
    FSC_OT_make_planar_single_face,
    FSC_PT_panel,
)


def register():
    for c in classes:
        bpy.utils.register_class(c)
    bpy.types.Scene.fsc_settings = bpy.props.PointerProperty(type=FSC_Settings)


def unregister():
    if hasattr(bpy.types.Scene, "fsc_settings"):
        del bpy.types.Scene.fsc_settings
    for c in reversed(classes):
        bpy.utils.unregister_class(c)


if __name__ == "__main__":
    register()
